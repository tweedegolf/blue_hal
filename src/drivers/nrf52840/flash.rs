//! Internal Flash controller for the NRF
use crate::{
    hal::flash::ReadWrite,
    nrf52840pac::NVMC,
    utilities::{
        bitwise::SliceBitSubset,
        memory::{self, IterableByOverlaps},
    },
};
use core::ops::{Add, Sub};
use nb::block;

pub struct NrfFlash {
    flash: NVMC,
}

#[derive(Copy, Clone, Debug)]
pub enum Error {
    MemoryNotReachable,
    MisalignedAccess,
}

#[derive(Default, Copy, Clone, Debug, PartialOrd, PartialEq, Ord, Eq)]
pub struct Address(pub u32);

impl Add<usize> for Address {
    type Output = Self;
    fn add(self, rhs: usize) -> Address {
        Address(self.0 + rhs as u32)
    }
}

impl Sub<usize> for Address {
    type Output = Self;
    fn sub(self, rhs: usize) -> Address {
        Address(self.0.saturating_sub(rhs as u32))
    }
}

impl Sub<Address> for Address {
    type Output = usize;
    fn sub(self, rhs: Address) -> usize {
        self.0.saturating_sub(rhs.0) as usize
    }
}

impl Into<usize> for Address {
    fn into(self) -> usize {
        self.0 as usize
    }
}

#[derive(Copy, Clone, Debug)]
struct Range(Address, Address);

/// Different address blocks as defined in [Table 5](../../../../../../../documentation/hardware/stm32f412_reference.pdf#page=58)
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Block {
    /// Main memory, where the application is written
    Main,
}

/// A memory map sector, with an associated block and an address range
/// The flash is divided into 256 pages of 4 kB
#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
struct Sector(u8);

const UNLOCK_KEYS: [u32; 2] = [0x45670123, 0xCDEF89AB];

#[cfg(feature = "nrf52840")]
const SECTOR_NUMBER: usize = 256;

const fn max_sector_size() -> usize {
    KB!(4)
}

#[non_exhaustive]
pub struct MemoryMap {}

impl MemoryMap {
    // Verifies that the memory map is consecutive and well formed
    fn is_sound(&self) -> bool {
        // trivial
        true
    }

    fn sectors() -> impl ExactSizeIterator<Item = Sector>
           + DoubleEndedIterator<Item = Sector>
           + Iterator<Item = Sector> {
        (std::u8::MIN..std::u8::MAX).map(Sector)
    }
}

impl Range {
    /// Sectors spanned by this range of addresses
    fn span(self) -> impl Iterator<Item = Sector> {
        let first = MemoryMap::sectors()
            .enumerate()
            .find_map(|(i, sector)| self.overlaps(&sector).then_some(i));
        let last = MemoryMap::sectors()
            .enumerate()
            .rev()
            .find_map(|(i, sector)| self.overlaps(&sector).then_some(i));
        match (first, last) {
            (Some(first), Some(last)) if (last >= first) => {
                MemoryMap::sectors().skip(first).take((last + 1) - first)
            }

            _ => MemoryMap::sectors().skip(0).take(0),
        }
    }

    const fn is_valid(self) -> bool {
        let Range(Address(start), Address(end)) = self;
        let after_map = start >= Sector(255).end().0;
        let before_map = end < Sector(0).end().0;
        let monotonic = end >= start;
        monotonic && !before_map && !after_map
    }

    fn overlaps(self, sector: &Sector) -> bool {
        (self.0 <= sector.start()) && (self.1 > sector.end())
            || (self.0 < sector.end()) && (self.1 >= sector.end())
            || (self.0 >= sector.start() && self.0 < sector.end())
            || (self.1 < sector.end() && self.1 >= sector.start())
    }

    /// Verify that all sectors spanned by this range are writable
    fn is_writable(self) -> bool {
        self.span().all(|x| Sector::is_writable(&x))
    }
}

impl memory::Region<Address> for Sector {
    fn contains(&self, address: Address) -> bool {
        (self.start() <= address) && (self.end() > address)
    }
}

impl Sector {
    const fn size() -> usize {
        KB!(4)
    }

    const fn start(&self) -> Address {
        Address(self.0 as u32 * Self::size() as u32)
    }
    const fn end(&self) -> Address {
        Address(self.start().0 + Self::size() as u32)
    }

    const fn new(index: u8) -> Self {
        Sector(index)
    }

    const fn is_writable(&self) -> bool {
        true
    }
    const fn is_in_main_memory_area(&self) -> bool {
        true
    }
}

impl NrfFlash {
    pub fn new(flash: NVMC) -> Self {
        Self { flash }
    }

    fn unlock(&mut self) -> nb::Result<(), Error> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        // enable writing
        self.flash.config.write(|w| {
            w.wen().wen();
            w
        });

        Ok(())
    }

    fn lock(&mut self) {
        // disable writing
        self.flash.config.write(|w| {
            w.wen().ren();
            w
        });
    }

    fn erase(&mut self, sector: &Sector) -> nb::Result<(), Error> {
        let page_number: u32 = sector.0 as u32;

        self.flash
            .erasepage_mut()
            .write(|w| unsafe { w.bits(page_number) });

        Ok(())
    }

    fn is_busy(&self) -> bool {
        // https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.1.pdf#page=27
        self.flash.ready.read().bits() == 0
    }

    fn write_bytes(
        &mut self,
        bytes: &[u8],
        sector: &Sector,
        address: Address,
    ) -> nb::Result<(), Error> {
        if (address < sector.start()) || (address + bytes.len() > sector.end()) {
            return Err(nb::Error::Other(Error::MisalignedAccess));
        }

        let words = bytes.chunks(4).map(|bytes| {
            u32::from_le_bytes([
                bytes.get(0).cloned().unwrap_or(0),
                bytes.get(1).cloned().unwrap_or(0),
                bytes.get(2).cloned().unwrap_or(0),
                bytes.get(3).cloned().unwrap_or(0),
            ])
        });

        block!(self.unlock())?;

        // TODO what does this do?
        // self.flash.cr.modify(|_, w| w.pg().set_bit());

        let base_address = address.0 as *mut u32;
        for (index, word) in words.enumerate() {
            // NOTE(Safety): Writing to a memory-mapped flash
            // directly is naturally unsafe. We have to trust that
            // the memory map is correct, and that these dereferences
            // won't cause a hardfault or overlap with our firmware.
            unsafe {
                *(base_address.add(index)) = word;
            }
        }
        self.lock();

        Ok(())
    }
}

impl ReadWrite for NrfFlash {
    type Error = Error;
    type Address = Address;

    fn range(&self) -> (Address, Address) {
        (Address(0), Address(256 * KB!(4)))
    }

    // erase user-writable memory
    fn erase(&mut self) -> nb::Result<(), Self::Error> {
        unsafe { &*NVMC::ptr() }
            .eraseall
            .write(|w| unsafe { w.bits(1) });

        Ok(())
    }

    fn write(&mut self, address: Address, bytes: &[u8]) -> nb::Result<(), Self::Error> {
        // address must be 32-bit aligned
        if address.0 % 4 != 0 {
            return Err(nb::Error::Other(Error::MisalignedAccess));
        }

        // trivial, all memory is writable
        let range = Range(address, Address(address.0 + bytes.len() as u32));
        if !range.is_writable() {
            return Err(nb::Error::Other(Error::MemoryNotReachable));
        }

        // Early yield if busy
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        for (block, sector, address) in MemoryMap::sectors().overlaps(bytes, address) {
            let sector_data = &mut [0u8; max_sector_size()][0..Sector::size()];
            let offset_into_sector = address.0.saturating_sub(sector.start().0) as usize;

            block!(self.read(sector.start(), sector_data))?;
            if block.is_subset_of(&sector_data[offset_into_sector..Sector::size()]) {
                // No need to erase the sector, as we can just flip bits off
                // (since our block is a bitwise subset of the sector)
                block!(self.write_bytes(block, &sector, address))?;
            } else {
                // We have to erase and rewrite any saved data alongside the new block
                block!(self.erase(&sector))?;
                sector_data
                    .iter_mut()
                    .skip(offset_into_sector)
                    .zip(block)
                    .for_each(|(byte, input)| *byte = *input);
                block!(self.write_bytes(sector_data, &sector, sector.start()))?;
            }
        }

        Ok(())
    }

    fn read(&mut self, address: Address, bytes: &mut [u8]) -> nb::Result<(), Self::Error> {
        let range = Range(address, Address(address.0 + bytes.len() as u32));
        if !range.is_writable() {
            Err(nb::Error::Other(Error::MemoryNotReachable))
        } else {
            let base = address.0 as *const u8;
            for (index, byte) in bytes.iter_mut().enumerate() {
                // NOTE(Safety) we are reading directly from raw memory locations,
                // which is inherently unsafe.
                *byte = unsafe { *(base.add(index)) };
            }
            Ok(())
        }
    }

    fn write_from_blocks<I: Iterator<Item = [u8; N]>, const N: usize>(
        &mut self,
        address: Self::Address,
        blocks: I,
    ) -> Result<(), Self::Error> {
        const TRANSFER_SIZE: usize = KB!(4);
        assert!(TRANSFER_SIZE % N == 0);
        let mut transfer_array = [0x00u8; TRANSFER_SIZE];
        let mut memory_index = 0usize;

        for block in blocks {
            let slice = &mut transfer_array
                [(memory_index % TRANSFER_SIZE)..((memory_index % TRANSFER_SIZE) + N)];
            slice.clone_from_slice(&block);
            memory_index += N;

            if memory_index % TRANSFER_SIZE == 0 {
                nb::block!(self.write(address + (memory_index - TRANSFER_SIZE), &transfer_array))?;
                transfer_array.iter_mut().for_each(|b| *b = 0x00u8);
            }
        }
        let remainder = &transfer_array[0..(memory_index % TRANSFER_SIZE)];
        nb::block!(self.write(address + (memory_index - remainder.len()), &remainder))?;
        Ok(())
    }

    fn label() -> &'static str {
        "stm32f4 flash (Internal)"
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn ranges_overlap_sectors_correctly() {
        let sector = Sector::new(Block::Reserved, Address(10), 10usize);
        assert!(Range(Address(10), Address(20)).overlaps(&sector));
        assert!(Range(Address(5), Address(15)).overlaps(&sector));
        assert!(Range(Address(15), Address(25)).overlaps(&sector));
        assert!(Range(Address(5), Address(25)).overlaps(&sector));
        assert!(Range(Address(12), Address(18)).overlaps(&sector));

        assert!(!Range(Address(0), Address(5)).overlaps(&sector));
        assert!(!Range(Address(20), Address(25)).overlaps(&sector));
    }

    //    #[test]
    //    fn ranges_span_the_correct_sectors() {
    //        let range = Range(Address(0x0801_1234), Address(0x0804_5678));
    //        let expected_sectors = &MEMORY_MAP.sectors[4..7];
    //
    //        assert_eq!(expected_sectors, range.span());
    //    }
    //
    //    #[test]
    //    fn map_shows_correct_writable_range() {
    //        let (start, end) = (MemoryMap::writable_start(), MemoryMap::writable_end());
    //        assert_eq!(start, MEMORY_MAP.sectors[4].start());
    //        assert_eq!(end, MEMORY_MAP.sectors[11].end());
    //    }
    //
    //    #[test]
    //    fn ranges_are_correctly_marked_writable() {
    //        let (start, size) = (Address(0x0801_0008), 48usize);
    //        let range = Range(start, Address(start.0 + size as u32));
    //        assert!(range.is_writable());
    //    }
}
