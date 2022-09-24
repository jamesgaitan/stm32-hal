/// MY_TODO Verify all configuration registers are accounted for
/// MY_TODO Try to put this file back or at least pull from baseline.rs
/// MY_TODO What is MCO and what is that extra dropdown into Cortex System Timer
use crate::{
    baseline::{ApbPrescaler, HclkPrescaler},
    clocks::SpeedError,
    pac::{self, FLASH, RCC},
    util::rcc_en_reset,
};

use cfg_if::cfg_if;

#[derive(Clone, Copy, PartialEq)]
pub enum PllSrc {
    None,
    Hsi,
    Hse(u32),
}

impl PllSrc {
    pub fn bits(&self) -> u8 {
        match self {
            Self::None => 0b00,
            Self::Hsi => 0b10,
            Self::Hse(_) => 0b11,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum InputSrc {
    Msi(MsiRange),
    Hsi,
    Hse(u32), // freq in Hz,
    Pll(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::Msi(_) => 0b00,
            Self::Hsi => 0b01,
            Self::Hse(_) => 0b10,
            Self::Pll(_) => 0b11,
        }
    }
}

/// Specify the range of MSI - this is effectively it's oscillation speed.
pub enum MsiRange {
    R65536 = 0b0000,
    R131072 = 0b0001,
    R262144 = 0b0010,
    R524288 = 0b0011,
    R1048K = 0b0100,
    R2097K = 0b0101, // default
    R4194K = 0b0110,
}

impl MsiRange {
    // Calculate the approximate frequency, in Hz.
    fn value(&self) -> u32 {
        match self {
            Self::R65536 => 65_536,
            Self::R131072 => 131_072,
            Self::R262144 => 262_144,
            Self::R524288 => 524_288,
            Self::R1048K => 1_048_000,
            Self::R2097K => 2_097_000,
            Self::R4194K => 4_194_000,
        }
    }
}

/// Configures the speeds an individual PLL
pub struct PllCfg {
    pub enabled: bool,
    pub pll_mul: Pllm,
    pub pll_div: Plld,
}

impl Default for PllCfg {
    fn default() -> Self {
        // todo: Different defaults for different variants.
        Self {
            enabled: true,
            pll_mul: Pllm::Mul2,
            pll_div: Plld::Div1,
        }
    }
}

impl PllCfg {
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }
}

pub enum Pllm {
    Mul1 = 0b0000,
    Mul2 = 0b0001,
    Mul3 = 0b0010,
    Mul4 = 0b0011,
    Mul5 = 0b0100,
    Mul6 = 0b0101,
    Mul7 = 0b0110,
    Mul8 = 0b0111,
    Mul9 = 0b1000,
}

impl Pllm {
    pub fn value(&self) -> u8 {
        match self {
            Self::Mul1 => 3,
            Self::Mul2 => 4,
            Self::Mul3 => 6,
            Self::Mul4 => 8,
            Self::Mul5 => 12,
            Self::Mul6 => 16,
            Self::Mul7 => 24,
            Self::Mul8 => 32,
            Self::Mul9 => 48,
        }
    }
}

pub enum Plld {
    Div1 = 0b01,
    Div2 = 0b10,
    Div3 = 0b11,
}

impl Plld {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 2,
            Self::Div2 => 3,
            Self::Div3 => 4,
        }
    }
}

/// Settings used to configure clocks. Create this struct by using its `Default::default()`
/// implementation, then modify as required, referencing your RM's clock tree,
/// or Stm32Cube IDE's interactive clock manager. Apply settings by running `.setup()`.
pub struct Clocks {
    /// The input source for the system and peripheral clocks.
    pub input_src: InputSrc,

    /// Enable and speed status for the main PLL
    pub pll: PllCfg,

    /// The value to divide SYSCLK by, to get systick and peripheral clocks. Also known as AHB divider
    pub hclk_prescaler: HclkPrescaler,

    /// The divider of HCLK to get the APB1 peripheral clock
    pub apb1_prescaler: ApbPrescaler,

    /// The divider of HCLK to get the APB2 peripheral clock
    pub apb2_prescaler: ApbPrescaler,
}

impl Clocks {
    /// MY_TODO
    /// Setup common and return Ok if the config is valid. Abort the setup if speeds
    /// are invalid.
    /// Use the STM32CubeIDE Clock Configuration tab to help identify valid configs.
    /// Use the `default()` implementation as a safe baseline.
    pub fn setup(&self) -> Result<(), SpeedError> {
        if let Err(e) = self.validate_speeds() {
            return Err(e);
        }

        let rcc = unsafe { &(*RCC::ptr()) };
        let flash = unsafe { &(*FLASH::ptr()) };

        // Enable and reset System Configuration Controller, ie for interrupts.
        // todo: Is this the right module to do this in?
        #[cfg(not(any(feature = "wb", feature = "wl")))] // todo: Do interrupts work without enabling syscfg on wb, which
                                                         // todo doesn't have this?
        rcc_en_reset!(apb2, syscfg, rcc);

        // Adjust flash wait states according to the HCLK frequency.
        // We need to do this before enabling PLL, or it won't enable.
        let sysclk = self.sysclk();

        cfg_if! {
            if #[cfg(feature = "wb")] {
                let hclk = sysclk / self.hclk4_prescaler.value() as u32;
            } else if #[cfg(feature = "wl")] {
                let hclk = sysclk / self.hclk3_prescaler.value() as u32;
            } else {
                let hclk = sysclk / self.hclk_prescaler.value() as u32;
            }
        }

        cfg_if! {
            if #[cfg(feature = "g4")] {
                if self.boost_mode {
                    // The sequence to switch from Range1 normal mode to Range1 boost mode is:
                    // 1. The system clock must be divided by 2 using the AHB prescaler before switching to a
                    // higher system frequency.
                    rcc.cfgr.modify(|_, w| unsafe { w.hpre().bits(HclkPrescaler::Div2 as u8) });
                    // 2. Clear the R1MODE bit is in the PWR_CR5 register.
                    let pwr = unsafe { &(*pac::PWR::ptr()) };
                    pwr.cr5.modify(|_, w| w.r1mode().clear_bit());
                }

                // (Remaining steps accomplished below)
                // 3. Adjust the number of wait states according to the new frequency target in range1 boost
                // mode
                // 4. Configure and switch to new system frequency.
                // 5. Wait for at least 1us and then reconfigure the AHB prescaler to get the needed HCLK
                // clock frequency.
            }
        }

        cfg_if! {
            if #[cfg(feature = "l4")] {  // RM section 3.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 16_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 32_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 48_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 64_000_000 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else {
                        w.latency().bits(WaitState::W4 as u8)
                    }
                });
            } else if #[cfg(feature = "l5")] {  // RM section 6.3.3
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 20_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 40_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 60_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else if hclk <= 80_000_000 {
                        w.latency().bits(WaitState::W3 as u8)
                    } else if hclk <= 100_000_000 {
                        w.latency().bits(WaitState::W4 as u8)
                    } else {
                        w.latency().bits(WaitState::W5 as u8)
                    }
                });
            } else if #[cfg(feature = "g0")] {  // G0. RM section 3.3.4
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 24_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 48_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else {
                        w.latency().bits(WaitState::W2 as u8)
                    }
                });
            } else if #[cfg(feature = "wb")] {  // WB. RM section 3.3.4, Table 4.
            // Note: This applies to HCLK4 HCLK. (See HCLK4 used above for hclk var.)
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 18_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 36_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else if hclk <= 54_000_000 {
                        w.latency().bits(WaitState::W2 as u8)
                    } else {
                        w.latency().bits(WaitState::W3 as u8)
                    }
                });
            } else if #[cfg(any(feature = "wb", feature = "wl"))] {  // WL. RM section 3.3.4, Table 5.
            // Note: This applies to HCLK3 HCLK. (See HCLK3 used above for hclk var.)
                flash.acr.modify(|_, w| unsafe {
                    if hclk <= 18_000_000 {
                        w.latency().bits(WaitState::W0 as u8)
                    } else if hclk <= 36_000_000 {
                        w.latency().bits(WaitState::W1 as u8)
                    } else {
                        w.latency().bits(WaitState::W2 as u8)
                    }
                });
            } else {  // G4. RM section 3.3.3
                flash.acr.modify(|_, w| unsafe {
                    if self.boost_mode {
                        // Vcore Range 1 boost mode
                        if hclk <= 34_000_000 {
                            w.latency().bits(WaitState::W0 as u8)
                        } else if hclk <= 68_000_000 {
                            w.latency().bits(WaitState::W1 as u8)
                        } else if hclk <= 102_000_000 {
                            w.latency().bits(WaitState::W2 as u8)
                        } else if hclk <= 136_000_000 {
                            w.latency().bits(WaitState::W3 as u8)
                        } else {
                            w.latency().bits(WaitState::W4 as u8)
                        }
                    } else {
                        // Vcore Range 1 normal mode.
                        if hclk <= 30_000_000 {
                            w.latency().bits(WaitState::W0 as u8)
                        } else if hclk <= 60_000_000 {
                            w.latency().bits(WaitState::W1 as u8)
                        } else if hclk <= 90_000_000 {
                            w.latency().bits(WaitState::W2 as u8)
                        } else if hclk <= 120_000_000 {
                            w.latency().bits(WaitState::W3 as u8)
                        } else {
                            w.latency().bits(WaitState::W4 as u8)
                        }
                    }
                });
            }
        }

        // Reference Manual, 6.2.5:
        // The device embeds 3 PLLs: PLL, PLLSAI1, PLLSAI2. Each PLL provides up to three
        // independent outputs. The internal PLLs can be used to multiply the HSI16, HSE or MSI
        // output clock frequency. The PLLs input frequency must be between 4 and 16 MHz. The
        // selected clock source is divided by a programmable factor PLLM from 1 to 8 to provide a
        // clock frequency in the requested input range. Refer to Figure 15: Clock tree (for
        // STM32L47x/L48x devices) and Figure 16: Clock tree (for STM32L49x/L4Ax devices) and
        // PLL configuration register (RCC_PLLCFGR).
        // The PLLs configuration (selection of the input clock and multiplication factor) must be done
        // before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
        // To modify the PLL configuration, proceed as follows:
        // 1. Disable the PLL by setting PLLON to 0 in Clock control register (RCC_CR).
        // 2. Wait until PLLRDY is cleared. The PLL is now fully stopped.
        // 3. Change the desired parameter.
        // 4. Enable the PLL again by setting PLLON to 1.
        // 5. Enable the desired PLL outputs by configuring PLLPEN, PLLQEN, PLLREN in PLL
        // configuration register (RCC_PLLCFGR).

        // Enable oscillators, and wait until ready.
        match self.input_src {
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => {
                // MSI initializes to the default clock source. Turn it off before
                // Adjusting its speed etc.
                rcc.cr.modify(|_, w| w.msion().clear_bit());
                while rcc.cr.read().msirdy().bit_is_set() {}

                rcc.cr.modify(|_, w| unsafe {
                    w.msirange().bits(range as u8);
                    #[cfg(not(any(feature = "wb", feature = "wl")))]
                    w.msirgsel().set_bit();
                    w.msion().set_bit()
                });
                // Wait for the MSI to be ready.
                while rcc.cr.read().msirdy().bit_is_clear() {}
                // todo: If LSE is enabled, calibrate MSI.
            }
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy().bit_is_clear() {}
            }
            InputSrc::Hsi => {
                rcc.cr.modify(|_, w| w.hsion().set_bit());
                while rcc.cr.read().hsirdy().bit_is_clear() {}
            }
            InputSrc::Pll(pll_src) => {
                // todo: PLL setup here is DRY with the HSE, HSI, and MSI setup above.
                match pll_src {
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi => {
                        rcc.cr.modify(|_, w| w.hsion().set_bit());
                        while rcc.cr.read().hsirdy().bit_is_clear() {}
                    }
                    PllSrc::None => {}
                }
            }
        }

        rcc.cr.modify(|_, w| w.hsebyp().bit(self.hse_bypass));

        rcc.cfgr.modify(|_, w| unsafe {
            w.sw().bits(self.input_src.bits());
            w.hpre().bits(self.hclk_prescaler as u8);
            w.ppre2().bits(self.apb2_prescaler as u8); // HCLK division for APB2.
            return w.ppre1().bits(self.apb1_prescaler as u8); // HCLK division for APB1
            return w.ppre().bits(self.apb1_prescaler as u8);
        });

        // Note that with this code setup, PLLSAI won't work properly unless using
        // the input source is PLL.
        if let InputSrc::Pll(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcc.cr.modify(|_, w| w.pllon().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcc.cr.read().pllrdy().bit_is_set() {}

            rcc.pllcfgr.modify(|_, w| unsafe {
                w.pllmul().bits(self.pll.pll_mul as u8);
                w.plldiv().bits(self.pll.pll_div as u8);
            });

            rcc.cr.modify(|_, w| w.pllon().set_bit());
            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

        /// MY_TODO
        // If we're not using the default clock source as input source or for PLL, turn it off.
        cfg_if! {
            if #[cfg(any(feature = "l4", feature = "l5"))] {
                match self.input_src {
                    InputSrc::Msi(_) => (),
                    InputSrc::Pll(pll_src) => {
                        match pll_src {
                        PllSrc::Msi(_) => (),
                            _ => {
                                rcc.cr.modify(|_, w| w.msion().clear_bit());
                            }
                        }
                    }
                    _ => {
                        rcc.cr.modify(|_, w| w.msion().clear_bit());
                   }
                }

            } else {
                 match self.input_src {
                    InputSrc::Hsi => (),
                    InputSrc::Pll(pll_src) => {
                        match pll_src {
                        PllSrc::Hsi => (),
                            _ => {
                                rcc.cr.modify(|_, w| w.hsion().clear_bit());
                            }
                        }
                    }
                    _ => {
                        rcc.cr.modify(|_, w| w.hsion().clear_bit());
                   }
                }
            }
        }

        Ok(())
    }

    /// MY_TODO
    /// Re-select input source; used after Stop and Standby modes, where the system reverts
    /// to MSI or HSI after wake.
    pub fn reselect_input(&self) {
        let rcc = unsafe { &(*RCC::ptr()) };

        // Re-select the input source; useful for changing input source, or reverting
        // from stop or standby mode. This assumes we're on a clean init,
        // or waking up from stop mode etc.

        match self.input_src {
            InputSrc::Hse(_) => {
                rcc.cr.modify(|_, w| w.hseon().set_bit());
                while rcc.cr.read().hserdy().bit_is_clear() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            InputSrc::Pll(pll_src) => {
                // todo: DRY with above.
                match pll_src {
                    PllSrc::Hse(_) => {
                        rcc.cr.modify(|_, w| w.hseon().set_bit());
                        while rcc.cr.read().hserdy().bit_is_clear() {}
                    }
                    PllSrc::Hsi => {
                        #[cfg(any(feature = "l4", feature = "l5"))]
                        // Generally reverts to MSI (see note below)
                        if let StopWuck::Msi = self.stop_wuck {
                            rcc.cr.modify(|_, w| w.hsion().set_bit());
                            while rcc.cr.read().hsirdy().bit_is_clear() {}
                        }
                        // If on G, we'll already be on HSI, so need to take action.
                    }
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    PllSrc::Msi(range) => {
                        // Range initializes to 4Mhz, so set that as well.
                        #[cfg(not(feature = "wb"))]
                        rcc.cr.modify(|_, w| unsafe {
                            w.msirange().bits(range as u8);
                            w.msirgsel().set_bit()
                        });
                        #[cfg(feature = "wb")]
                        rcc.cr
                            .modify(|_, w| unsafe { w.msirange().bits(range as u8) });

                        if let StopWuck::Hsi = self.stop_wuck {
                            rcc.cr.modify(|_, w| w.msion().set_bit());

                            while rcc.cr.read().msirdy().bit_is_clear() {}
                        }
                    }
                    PllSrc::None => (),
                }

                rcc.cr.modify(|_, w| w.pllon().clear_bit());
                while rcc.cr.read().pllrdy().bit_is_set() {}

                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });

                rcc.cr.modify(|_, w| w.pllon().set_bit());
                while rcc.cr.read().pllrdy().bit_is_clear() {}
            }
            InputSrc::Hsi => {
                {
                    // (This note applies to L4 and L5 only)
                    // From L4 Reference Manual, RCC_CFGR register section:
                    // "Configured by HW to force MSI oscillator selection when exiting Standby or Shutdown mode.
                    // Configured by HW to force MSI or HSI16 oscillator selection when exiting Stop mode or in
                    // case of failure of the HSE oscillator, depending on STOPWUCK value."

                    // So, if stopwuck is at its default value of MSI, we need to re-enable HSI,
                    // and re-select it. Otherwise, take no action. Reverse for MSI-reselection.
                    // For G, we already are using HSI, so need to take action either.
                    #[cfg(not(any(feature = "g0", feature = "g4")))]
                    if let StopWuck::Msi = self.stop_wuck {
                        rcc.cr.modify(|_, w| w.hsion().set_bit());
                        while rcc.cr.read().hsirdy().bit_is_clear() {}

                        rcc.cfgr
                            .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
                    }
                }
            }
            #[cfg(not(any(feature = "g0", feature = "g4")))]
            InputSrc::Msi(range) => {
                // Range initializes to 4Mhz, so set that as well.
                #[cfg(not(feature = "wb"))]
                rcc.cr.modify(|_, w| unsafe {
                    w.msirange().bits(range as u8);
                    w.msirgsel().set_bit()
                });
                #[cfg(feature = "wb")]
                rcc.cr
                    .modify(|_, w| unsafe { w.msirange().bits(range as u8) });

                if let StopWuck::Hsi = self.stop_wuck {
                    rcc.cr.modify(|_, w| w.msion().set_bit());
                    while rcc.cr.read().msirdy().bit_is_clear() {}

                    rcc.cfgr
                        .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
                }
            }
            #[cfg(feature = "g0")]
            InputSrc::Lsi => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());
                while rcc.csr.read().lsirdy().bit_is_clear() {}
                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
            #[cfg(feature = "g0")]
            InputSrc::Lse => {
                rcc.bdcr.modify(|_, w| w.lseon().set_bit());
                while rcc.bdcr.read().lserdy().bit_is_clear() {}
                rcc.cfgr
                    .modify(|_, w| unsafe { w.sw().bits(self.input_src.bits()) });
            }
        }
    }

    /// Use this to change the MSI speed. Run this only if your clock source is MSI.
    /// Ends in a state with MSI on at the new speed, and HSI off.
    pub fn change_msi_speed(&mut self, range: MsiRange) {
        let rcc = unsafe { &(*RCC::ptr()) };

        match self.input_src {
            InputSrc::Msi(_) => (),
            _ => panic!("Only change MSI speed using this function if MSI is the input source."),
        }

        // RM: "`"Warning: MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1).
        // MSIRANGE must NOT be modified when MSI is ON and NOT ready (MSION=1 and MSIRDY=0)"
        // So, we can change MSI range while it's running.
        while rcc.cr.read().msirdy().bit_is_clear() {}

        rcc.cr
            .modify(|_, w| unsafe { w.msirange().bits(range as u8).msirgsel().set_bit() });

        // Update our config to reflect the new speed.
        self.input_src = InputSrc::Msi(range);
    }

    /// Get the sysclock frequency, in hz.
    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Pll(pll_src) => {
                let input_freq = match pll_src {
                    PllSrc::Hsi => 16_000_000,
                    PllSrc::Hse(freq) => freq,
                    PllSrc::None => unimplemented!(),
                };
                input_freq * self.pll.pll_mul.value() as u32 / self.pll.pll_div.value() as u32
            }
            InputSrc::Msi(range) => range.value() as u32,
            InputSrc::Hsi => 16_000_000,
            InputSrc::Hse(freq) => freq,
        }
    }

    /// Check if the PLL is enabled. This is useful if checking whether to re-enable the PLL
    /// after exiting Stop or Standby modes, eg so you don't re-enable if it was already re-enabled
    /// in a different context. eg:
    /// ```
    /// if !clock_cfg.pll_is_enabled() {
    ///     clock_cfg.reselect_input();
    ///}
    ///```
    pub fn pll_is_enabled(&self) -> bool {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.cr.read().pllon().bit_is_set()
    }

    /// Get the HCLK frequency, in hz
    pub fn hclk(&self) -> u32 {
        self.sysclk() / self.hclk_prescaler.value() as u32
    }

    /// Get the systick frequency, in  hz
    pub fn systick(&self) -> u32 {
        self.hclk()
    }

    /// Get the APB1 frequency, in hz
    pub fn apb1(&self) -> u32 {
        self.hclk() / self.apb1_prescaler.value() as u32
    }

    /// Get the frequency used by APB1 timers, in hz
    pub fn apb1_timer(&self) -> u32 {
        // 1. If the APB prescaler equals 1, the timer clock frequencies are set to the same
        // frequency as that of the APB domain.
        // 2. Otherwise, they are set to twice (×2) the frequency of the APB domain.
        if let ApbPrescaler::Div1 = self.apb1_prescaler {
            self.apb1()
        } else {
            self.apb1() * 2
        }
    }

    pub fn apb2(&self) -> u32 {
        self.hclk() / self.apb2_prescaler.value() as u32
    }

    pub fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb2_prescaler {
            self.apb2()
        } else {
            self.apb2() * 2
        }
    }

    pub fn validate_speeds(&self) -> Result<(), SpeedError> {
        let max_clock = 32_000_000;

        if self.sysclk() > max_clock {
            return Err(SpeedError::new("Sysclk out of limits"));
        }

        if self.hclk() > max_clock {
            return Err(SpeedError::new("Hclk out of limits"));
        }

        if self.apb1() > max_clock {
            return Err(SpeedError::new("Apb1 out of limits"));
        }

        if self.apb2() > max_clock {
            return Err(SpeedError::new("Apb2 out of limits"));
        }

        Ok(())
    }
}

impl Default for Clocks {
    /// This default configures clocks with a HSI, with system and peripheral clocks at full rated speed (32MHz)
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::Hsi),
            pll: PllCfg::default(),
            hclk_prescaler: HclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div1,
            apb2_prescaler: ApbPrescaler::Div1,
        }
    }
}
