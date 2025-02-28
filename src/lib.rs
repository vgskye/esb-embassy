#![no_std]

use core::{
    arch::asm,
    marker::PhantomData,
    mem::{offset_of, ManuallyDrop},
    ptr::null_mut,
    sync::atomic::{compiler_fence, AtomicPtr, Ordering},
    task::Poll,
};

use embassy_nrf::pac::radio::vals::State as RadioState;
use embassy_nrf::{
    interrupt,
    pac::radio::vals::{Crcstatus, Dtx, Endian, Len, Map, Mode, Ru, Skipaddr},
    peripherals,
    ppi::ConfigurableChannel,
    Peripheral, PeripheralRef,
};
use embassy_nrf::{
    interrupt::typelevel::Interrupt,
    pac::radio::vals::{Crcinc, Plen, Txpower},
};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Instant};

pub struct Config {
    pub bitrate: Bitrate,
    pub crc: Crc,
    pub tx_output_power: embassy_nrf::pac::radio::vals::Txpower,
    pub retransmit_delay: Duration,
    pub retransmit_count: u16,
    pub max_payload_length: u8,
    pub use_fast_ramp_up: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bitrate: Bitrate::Esb2Mbps,
            crc: Crc::Crc16Bit,
            tx_output_power: Txpower::_0_DBM,
            retransmit_delay: Duration::from_micros(600),
            retransmit_count: 3,
            max_payload_length: 252,
            use_fast_ramp_up: true,
        }
    }
}

pub enum Bitrate {
    Esb1Mbps,
    Esb2Mbps,
    Ble1Mbps,
}

pub enum Crc {
    Crc16Bit,
    Crc8Bit,
    Off,
}

pub const MAX_PAYLOAD_LEN: u8 = 252;

impl Default for Packet {
    fn default() -> Self {
        Self::new()
    }
}

/// Enhanced ShockBurst radio driver for PTX.
pub struct Radio<'d, T: Instance> {
    _p: PeripheralRef<'d, T>,
    config: Config,
    last_pid: u8,
}

impl<'d, T: Instance> Radio<'d, T> {
    /// Create a new IEEE 802.15.4 radio driver.
    ///
    /// # Panics
    ///
    /// This function panics if `config.payload_length` is above `MAX_PAYLOAD_LEN`.
    pub fn new(
        radio: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let radio = radio.into_ref();

        assert!(config.max_payload_length <= MAX_PAYLOAD_LEN);

        let r = T::regs();

        // cycle POWER
        r.power().write(|w| w.set_power(false));
        r.power().write(|w| w.set_power(true));

        r.frequency().write(|w| {
            w.set_map(Map::DEFAULT);
            w.set_frequency(2);
        });

        r.txpower().write(|w| w.set_txpower(config.tx_output_power));

        r.mode().write(|w| {
            w.set_mode(match config.bitrate {
                Bitrate::Esb1Mbps => Mode::NRF_1MBIT,
                Bitrate::Esb2Mbps => Mode::NRF_2MBIT,
                Bitrate::Ble1Mbps => Mode::BLE_1MBIT,
            })
        });

        r.pcnf0().write(|w| {
            w.set_lflen(if config.max_payload_length >= 32 {
                8
            } else {
                6
            });
            w.set_s0len(false);
            w.set_s1len(3);
            w.set_cilen(0);
            w.set_plen(Plen::_8BIT);
            w.set_crcinc(Crcinc::EXCLUDE);
            w.set_termlen(0);
        });

        r.pcnf1().write(|w| {
            w.set_maxlen(config.max_payload_length);
            w.set_balen(4);
            w.set_endian(Endian::BIG);
            w.set_statlen(0);
            w.set_whiteen(false);
        });

        r.base0().write(|w| *w = (0xE7E7E7E7u32).reverse_bits());
        r.base1().write(|w| *w = (0xC2C2C2C2u32).reverse_bits());

        r.prefix0().write(|w| {
            w.set_ap0((0xE7u8).reverse_bits());
            w.set_ap1((0xC2u8).reverse_bits());
            w.set_ap2((0xC3u8).reverse_bits());
            w.set_ap3((0xC4u8).reverse_bits());
        });
        r.prefix1().write(|w| {
            w.set_ap4((0xC5u8).reverse_bits());
            w.set_ap5((0xC6u8).reverse_bits());
            w.set_ap6((0xC7u8).reverse_bits());
            w.set_ap7((0xC8u8).reverse_bits());
        });

        r.crccnf().write(|w| {
            w.set_len(match config.crc {
                Crc::Crc16Bit => Len::TWO,
                Crc::Crc8Bit => Len::ONE,
                Crc::Off => Len::DISABLED,
            });
            w.set_skipaddr(Skipaddr::INCLUDE);
        });

        r.crcpoly().write(|w| {
            w.set_crcpoly(match config.crc {
                Crc::Crc16Bit => 0b10001000000100001,
                Crc::Crc8Bit => 0b100000111,
                Crc::Off => 0,
            })
        });

        r.crcinit().write(|w| {
            w.set_crcinit(match config.crc {
                Crc::Crc16Bit => 0xFFFF,
                Crc::Crc8Bit => 0xFF,
                Crc::Off => 0,
            })
        });

        r.modecnf0().write(|w| {
            w.set_ru(if config.use_fast_ramp_up {
                Ru::FAST
            } else {
                Ru::DEFAULT
            });
            w.set_dtx(Dtx::B1);
        });

        r.intenclr().write(|w| w.0 = 0xFFFFFFFF);

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            _p: radio,
            config,
            last_pid: 0,
        }
    }

    pub async fn try_send_no_ack(&mut self, packet: &mut Packet, address: u8) -> Result<(), Error> {
        if address >= 8 {
            return Err(Error::InvalidAddress);
        }
        let our_pid = self.last_pid;
        self.last_pid = (self.last_pid + 1) & 0b11;
        packet.buffer[1] = (our_pid << 1) | 0b1;

        let r = T::regs();
        let s = unsafe { T::state() };

        r.packetptr()
            .write_value((packet as *mut Packet).addr() as u32); // we're always on a 32-bit core, this is a fucking driver code
        r.txaddress().write(|w| w.set_txaddress(address));
        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_end_disable(true);
        });
        r.intenset().write(|w| {
            w.set_disabled(true);
        });
        r.events_disabled().write_value(0);

        let dropper = OnDrop::new(|| self.disable());

        dma_start_fence();

        r.tasks_txen().write_value(1);
        core::future::poll_fn(|cx| {
            s.event_waker.register(cx.waker());
            if r.events_disabled().read() != 0 {
                r.events_disabled().write_value(0);
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        dma_end_fence();
        dropper.defuse();

        Ok(())
    }

    pub async fn try_send_with_ack(
        &mut self,
        timer: embassy_nrf::pac::timer::Timer,
        packet: &mut Packet,
        address: u8,
        ppi_channels: (
            &mut impl ConfigurableChannel,
            &mut impl ConfigurableChannel,
            &mut impl ConfigurableChannel,
        ),
    ) -> Result<Packet, Error> {
        if address >= 8 {
            return Err(Error::InvalidAddress);
        }

        let our_pid = self.last_pid;
        self.last_pid = (self.last_pid + 1) & 0b11;
        packet.buffer[1] = our_pid << 1;

        let mut recv_packet = (
            Packet::new(),
            [
                ppi_channels.0.number(),
                ppi_channels.1.number(),
                ppi_channels.2.number(),
            ],
            timer,
        );

        let r = T::regs();
        let s = unsafe { T::state() };

        for _ in 0..self.config.retransmit_count {
            timer.shorts().write(|w| {
                w.set_compare_stop(0, true);
                w.set_compare_clear(0, true);
            });
            timer.cc(0).write_value(300);
            timer.prescaler().write(|w| w.set_prescaler(4));
            timer.events_compare(0).write_value(0);
            timer.tasks_clear().write_value(1);

            r.packetptr()
                .write_value((packet as *mut Packet).addr() as u32);
            r.txaddress().write(|w| w.set_txaddress(address));
            r.rxaddresses().write(|w| w.0 = 1 << address);
            r.shorts().write(|w| {
                w.set_ready_start(true);
            });

            r.events_end().write_value(0);

            s.tx_ack_packet_ptr
                .store(core::ptr::from_mut(&mut recv_packet), Ordering::SeqCst);

            r.intenset().write(|w| {
                w.set_end(true);
            });

            let retransmit = Instant::now() + self.config.retransmit_delay;

            let dropper = OnDrop::new(|| {
                timer.tasks_stop().write_value(1);
                self.disable()
            });

            dma_start_fence();

            r.tasks_txen().write_value(1);
            core::future::poll_fn(|cx| {
                s.event_waker.register(cx.waker());
                if r.events_disabled().read() != 0 {
                    r.events_disabled().write_value(0);
                    return Poll::Ready(());
                }
                Poll::Pending
            })
            .await;

            dma_end_fence();
            dropper.defuse();

            if r.events_end().read() == 1 && r.crcstatus().read().crcstatus() == Crcstatus::CRCOK {
                return Ok(recv_packet.0);
            }

            embassy_time::Timer::at(retransmit).await;
        }

        // we absolutely, positively want this dying here and not a cycle earlier
        #[allow(clippy::drop_non_drop)]
        #[allow(dropping_copy_types)]
        drop(core::hint::black_box(recv_packet));
        Err(Error::MaxRetryExceeded)
    }

    pub async fn try_recv(&mut self, address_mask: u8, reply_packet: &mut Packet) -> Packet {
        let mut recv_packet = Packet::new();

        let r = T::regs();
        let s = unsafe { T::state() };

        r.packetptr()
            .write_value(core::ptr::from_mut(&mut recv_packet).addr() as u32);
        r.rxaddresses().write(|w| w.0 = address_mask as u32);
        r.shorts().write(|w| {
            w.set_ready_start(true);
        });
        r.events_crcok().write_value(0);
        r.events_end().write_value(0);

        s.rx_ack_packet_ptr
            .store(core::ptr::from_mut(reply_packet), Ordering::SeqCst);

        r.intenset().write(|w| w.set_crcok(true));

        let dropper = OnDrop::new(|| self.disable());

        dma_start_fence();

        r.tasks_rxen().write_value(1);
        core::future::poll_fn(|cx| {
            s.event_waker.register(cx.waker());
            if r.events_end().read() != 0 {
                r.events_end().write_value(0);
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        dma_end_fence();
        dropper.defuse();

        recv_packet
    }

    /// Moves the radio from any state to the DISABLED state
    fn disable(&mut self) {
        let r = T::regs();
        let s = unsafe { T::state() };
        s.tx_ack_packet_ptr.store(null_mut(), Ordering::SeqCst);
        s.rx_ack_packet_ptr.store(null_mut(), Ordering::SeqCst);
        // See figure 110 in nRF52840-PS
        loop {
            match r.state().read().state() {
                RadioState::DISABLED => return,
                // idle or ramping up
                RadioState::RX_RU
                | RadioState::RX_IDLE
                | RadioState::TX_RU
                | RadioState::TX_IDLE => {
                    r.tasks_disable().write_value(1);
                    self.wait_for_radio_state(RadioState::DISABLED);
                    dma_end_fence();
                    return;
                }
                // ramping down
                RadioState::RX_DISABLE | RadioState::TX_DISABLE => {
                    self.wait_for_radio_state(RadioState::DISABLED);
                    dma_end_fence();
                    return;
                }
                // cancel ongoing transfer or ongoing CCA
                RadioState::RX => {
                    r.tasks_ccastop().write_value(1);
                    r.tasks_stop().write_value(1);
                    self.wait_for_radio_state(RadioState::RX_IDLE);
                }
                RadioState::TX => {
                    r.tasks_stop().write_value(1);
                    self.wait_for_radio_state(RadioState::TX_IDLE);
                }
                _ => unreachable!(),
            }
        }
    }
    /// Waits until the radio state matches the given `state`
    fn wait_for_radio_state(&self, state: RadioState) {
        let r = T::regs();
        while r.state().read().state() != state {
            unsafe {
                asm!("nop");
            }
        }
    }
}

struct OnDrop<T: FnOnce()>(ManuallyDrop<T>);

impl<T: FnOnce()> OnDrop<T> {
    fn new(on_drop: T) -> Self {
        Self(ManuallyDrop::new(on_drop))
    }

    fn defuse(mut self) {
        unsafe { ManuallyDrop::drop(&mut self.0) };
        core::mem::forget(self);
    }
}

impl<T: FnOnce()> Drop for OnDrop<T> {
    fn drop(&mut self) {
        (unsafe { ManuallyDrop::take(&mut self.0) })()
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidAddress,
    MaxRetryExceeded,
}

/// An Enhanced ShockBurst packet
#[repr(transparent)]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet {
    buffer: [u8; Self::SIZE],
}

impl Packet {
    const DATA: core::ops::RangeFrom<usize> = 2..;

    const SIZE: usize = 1 /* LENGTH */ + 1 /* S1 */ + MAX_PAYLOAD_LEN as usize;

    /// Returns an empty packet (length = 0)
    pub const fn new() -> Self {
        Self {
            buffer: [0; Self::SIZE],
        }
    }

    /// Fills the packet payload with given `src` data
    ///
    /// # Panics
    ///
    /// This function panics if `src` is larger than `MAX_PAYLOAD_LEN`
    pub fn copy_from_slice(&mut self, src: &[u8]) {
        assert!(src.len() <= MAX_PAYLOAD_LEN as usize);
        let len = src.len() as u8;
        self.buffer[Self::DATA][..len as usize].copy_from_slice(&src[..len.into()]);
        self.set_len(len);
    }

    /// Returns the size of this packet's payload
    pub fn len(&self) -> u8 {
        self.buffer[0]
    }

    pub fn is_empty(&self) -> bool {
        self.buffer[0] == 0
    }

    /// Changes the size of the packet's payload
    ///
    /// # Panics
    ///
    /// This function panics if `len` is larger than `MAX_PAYLOAD_LEN`
    pub fn set_len(&mut self, len: u8) {
        assert!(len <= MAX_PAYLOAD_LEN);
        self.buffer[0] = len;
    }

    pub fn pid(&self) -> u8 {
        (self.buffer[1] >> 1) & 0b11
    }

    pub fn no_ack(&self) -> bool {
        (self.buffer[1] & 1) == 1
    }
}

impl core::ops::Deref for Packet {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.buffer[Self::DATA][..self.len() as usize]
    }
}

impl core::ops::DerefMut for Packet {
    fn deref_mut(&mut self) -> &mut [u8] {
        let len = self.len();
        &mut self.buffer[Self::DATA][..len as usize]
    }
}

/// Interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();
        // clear all interrupts
        r.intenclr().write(|w| w.0 = 0xffff_ffff);
        let ptr = s.tx_ack_packet_ptr.swap(null_mut(), Ordering::SeqCst);
        let ptr_rx = s.rx_ack_packet_ptr.swap(null_mut(), Ordering::SeqCst);
        if ptr.is_null() && ptr_rx.is_null() {
            s.event_waker.wake();
        } else if ptr.is_null() {
            let recvd_packet = &*(r.packetptr().read() as *const Packet);
            if recvd_packet.buffer[1] & 1 == 1 {
                s.event_waker.wake();
            } else {
                r.shorts().write(|w| {
                    w.set_ready_start(true);
                    w.set_end_disable(true);
                });
                r.packetptr().write_value(ptr_rx.addr() as u32);
                r.events_disabled().write_value(0);
                r.intenset().write(|w| w.set_disabled(true));

                dma_start_fence();
                r.tasks_txen().write_value(1);
            }
        } else {
            r.shorts().write(|w| {
                w.set_ready_start(true);
                w.set_end_disable(true);
            });
            r.packetptr().write_value(
                ptr.byte_offset(offset_of!(CallbackPassPtx, 0) as isize)
                    .addr() as u32,
            );

            let ppi = embassy_nrf::pac::PPI;

            let ppi_channels =
                &mut *(ptr.byte_offset(offset_of!(CallbackPassPtx, 1) as isize) as *mut [usize; 3]);

            let timer = &mut *(ptr.byte_offset(offset_of!(CallbackPassPtx, 2) as isize)
                as *mut embassy_nrf::pac::timer::Timer);

            r.events_ready().write_value(0);
            r.events_address().write_value(0);
            timer.events_compare(0).write_value(0);

            ppi.chenclr().write(|w| {
                w.set_ch(ppi_channels[0], true);
                w.set_ch(ppi_channels[1], true);
                w.set_ch(ppi_channels[2], true);
            });

            ppi.fork(ppi_channels[0]).tep().write_value(0);
            ppi.ch(ppi_channels[0])
                .eep()
                .write_value(r.events_ready().as_ptr().addr() as u32);
            ppi.ch(ppi_channels[0])
                .tep()
                .write_value(timer.tasks_start().as_ptr().addr() as u32);

            ppi.fork(ppi_channels[1]).tep().write_value(0);
            ppi.ch(ppi_channels[1])
                .eep()
                .write_value(r.events_address().as_ptr().addr() as u32);
            ppi.ch(ppi_channels[1])
                .tep()
                .write_value(timer.tasks_stop().as_ptr().addr() as u32);

            ppi.fork(ppi_channels[2]).tep().write_value(0);
            ppi.ch(ppi_channels[2])
                .eep()
                .write_value(timer.events_compare(0).as_ptr().addr() as u32);
            ppi.ch(ppi_channels[2])
                .tep()
                .write_value(r.tasks_disable().as_ptr().addr() as u32);

            ppi.chenset().write(|w| {
                w.set_ch(ppi_channels[0], true);
                w.set_ch(ppi_channels[1], true);
                w.set_ch(ppi_channels[2], true);
            });

            r.events_end().write_value(0);
            r.events_disabled().write_value(0);
            r.intenset().write(|w| w.set_disabled(true));

            dma_start_fence();

            r.tasks_rxen().write_value(1);
        }
    }
}

type CallbackPassPtx = (Packet, [usize; 3], embassy_nrf::pac::timer::Timer);
type CallbackPassPrx = Packet;

pub(crate) struct State {
    tx_ack_packet_ptr: AtomicPtr<CallbackPassPtx>,
    rx_ack_packet_ptr: AtomicPtr<CallbackPassPrx>,
    event_waker: AtomicWaker,
}
impl State {
    pub(crate) const fn new() -> Self {
        Self {
            tx_ack_packet_ptr: AtomicPtr::new(null_mut()),
            rx_ack_packet_ptr: AtomicPtr::new(null_mut()),
            event_waker: AtomicWaker::new(),
        }
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> embassy_nrf::pac::radio::Radio;
    unsafe fn state() -> &'static State;
}

/// Radio peripheral instance.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl SealedInstance for peripherals::RADIO {
    fn regs() -> embassy_nrf::pac::radio::Radio {
        embassy_nrf::pac::RADIO
    }

    unsafe fn state() -> &'static State {
        static STATE: State = State::new();
        &STATE
    }
}

impl Instance for peripherals::RADIO {
    type Interrupt = embassy_nrf::interrupt::typelevel::RADIO;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}

/// NOTE must be followed by a volatile write operation
fn dma_start_fence() {
    compiler_fence(Ordering::Release);
}

/// NOTE must be preceded by a volatile read operation
fn dma_end_fence() {
    compiler_fence(Ordering::Acquire);
}
