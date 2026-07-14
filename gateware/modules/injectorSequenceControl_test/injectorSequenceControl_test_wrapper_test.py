import cocotb
from cocotb.clock import Clock
from cocotb.triggers import (
    RisingEdge,
    ClockCycles,
    with_timeout,
    Combine,
    SimTimeoutError,
)
from cocotb.handle import Immediate
from dataclasses import dataclass
import random
import logging
import numpy as np


class Timing:
    @staticmethod
    def arb_2_coinc_idx(ar_bucket):
        return (5 * ar_bucket) % 304

    @staticmethod
    def coinc_idx_2_coinc_term(coinc_idx):
        return (43 * coinc_idx) % 125

    @staticmethod
    def align_idx_2_align_term(br_ar_align):
        return (72 * br_ar_align) % 125

    @staticmethod
    def arb_2_coinc_term(ar_bucket):
        coinc_idx = Timing.arb_2_coinc_idx(ar_bucket)
        return Timing.coinc_idx_2_coinc_term(coinc_idx)

    @staticmethod
    def terms_2_br_bucket(coinc_term, align_term):
        return (coinc_term + align_term) % 125

    @staticmethod
    def br_bucket_2_inj_delay(br_bucket):
        return (94 * br_bucket) % 125

    @staticmethod
    def rf_coinc_2_extr_delay(rf_coinc):
        return 152 * rf_coinc

@dataclass
class InjectionParameters:
    inj_delay : int
    extr_delay : int
    align_idx : int
    coinc_idx : int
    br_bucket : int


class TB:
    def __init__(self, dut):
        dut._log.setLevel(logging.INFO)
        self.dut = dut

        self.dut.sysGPIO_OUT.value = Immediate(0)
        self.dut.sysCsrStrobe.value = Immediate(0)
        self.dut.sysCsrAlignStrobe.value = Immediate(0)
        self.dut.sysCsrTargetStrobe.value = Immediate(0)

        cocotb.start_soon(Clock(dut.sysClk, 10, unit="ns").start())
        cocotb.start_soon(Clock(dut.evgTxClk, 8, unit="ns").start())

    async def wait_for_heartbeat(self):
        await RisingEdge(self.dut.evgHeartbeatAlign)

    def is_clk_gen_synched(self):
        width = len(self.dut.clkGenSynceds)
        expected_val = (1 << width) - 1

        if (
            self.dut.clkGenSynceds.value.is_resolvable
            and self.dut.clkGenSynceds.value.to_unsigned() == expected_val
        ):
            return True
        else:
            return False

    async def wait_sync(self):
        await self.wait_for_heartbeat()
        await self.wait_for_heartbeat()
        await RisingEdge(self.dut.evgTxClk)
        await RisingEdge(self.dut.evgTxClk)

        assert self.is_clk_gen_synched(), f"Clock generation is not synchronized"

    async def monitor_sync(self):
        while True:
            await RisingEdge(self.dut.evgTxClk)

            assert self.is_clk_gen_synched(), f"Clock generation is not synchronized"

    async def wait_for_powerline(self):
        await RisingEdge(self.dut.evgPowerlineMon)

    async def wait_for_alignment(self, idx=0):
        sig = self.dut.evgAlignCounterDone
        prev_val = sig.value[idx]

        while True:
            await sig.value_change
            curr_val = sig.value[idx]

            if prev_val == 0 and curr_val == 1:
                break

            prev_val = curr_val

    async def wait_for_coinc_idx(self, coinc_idx):
        current_coinc_idx, _ = await self.read_current_counters()

        while current_coinc_idx != coinc_idx:
            current_coinc_idx, _ = await self.read_current_counters()

    async def _write_csr(self, strobe_name, value):
        stb = getattr(self.dut, strobe_name)

        await RisingEdge(self.dut.sysClk)
        self.dut.sysGPIO_OUT.value = value
        stb.value = 1

        await RisingEdge(self.dut.sysClk)
        stb.value = 0

    async def write_target_csr(self, value):
        await self._write_csr("sysCsrTargetStrobe", value)

    async def write_align_csr(self, value):
        await self._write_csr("sysCsrAlignStrobe", value)

    async def write_csr(self, value):
        await self._write_csr("sysCsrStrobe", value)

    def _read_csr(self, reg_name):
        reg = getattr(self.dut, reg_name)

        return reg.value.to_unsigned()

    async def read_status(self):
        await RisingEdge(self.dut.sysClk)
        return self._read_csr("sysStatus")

    async def read_align_status(self):
        await RisingEdge(self.dut.sysClk)
        return self._read_csr("sysAlignStatus")

    async def read_target_status(self):
        await RisingEdge(self.dut.sysClk)
        return self._read_csr("sysTargetStatus")

    async def read_target_status2(self):
        await RisingEdge(self.dut.sysClk)
        return self._read_csr("sysTargetStatus2")

    async def read_inj_delays(self):
        await RisingEdge(self.dut.evgTxClk)
        return (
            self.dut.evgInjDelay.value.to_unsigned(),
            self.dut.evgExtrDelay.value.to_unsigned(),
        )

    async def read_current_counters(self):
        await RisingEdge(self.dut.evgTxClk)
        return (
            self.dut.evgRFCoincCountMon.value.to_unsigned(),
            self.dut.evgRFAlignCountMon.value.to_unsigned(),
        )

    def gen_random_ar_bucket(self):
        ar_bucket = random.randint(0, 303)
        return ar_bucket

    async def wait_random(self):
        num_cycles = random.randint(0, 1000)
        await ClockCycles(self.dut.sysClk, num_cycles)
        return num_cycles

    async def injection_check_fsm(self, ar_bucket):
        self.dut._log.info(f"inj_check: AR bucket selection: {ar_bucket}")

        coinc_idx = Timing.arb_2_coinc_idx(ar_bucket)
        coinc_term = Timing.arb_2_coinc_term(ar_bucket)

        self.dut._log.info(f"inj_check: Coincidence index: {coinc_idx}")
        self.dut._log.info(f"inj_check: Coincidence term: {coinc_term}")

        # Mimic internal FSM
        assert self.dut.evgSeqBusy.value == 0, f"FAIL: inj_check: Injection FSM is busy"

        # Wait for it to start
        self.dut._log.info(f"inj_check: Waiting for sequence request...")
        await RisingEdge(self.dut.evgSeqBusy)

        # Wait for powerline trigger
        self.dut._log.info(f"inj_check: Waiting for powerline...")
        await self.wait_for_powerline()

        # Wait for alignment clock
        self.dut._log.info(f"inj_check: Waiting for alignment...")
        await self.wait_for_alignment()

        # Wait for coincidence index to match
        self.dut._log.info(f"inj_check: Waiting for coincidence index: {coinc_idx}...")
        await self.wait_for_coinc_idx(coinc_idx)

        # Get current counters
        coinc_count, align_count = await self.read_current_counters()

        assert (
            coinc_idx == coinc_count
        ), f"FAIL: inj_check: Current coincidence count ({coinc_count}) "
        f"differs from requested index ({coinc_idx})"

        # Calculate expected BR bucket
        expected_align_term = Timing.align_idx_2_align_term(align_count)
        expected_br_bucket = Timing.terms_2_br_bucket(
            coinc_term, expected_align_term
        )
        expected_inj_delay = Timing.br_bucket_2_inj_delay(expected_br_bucket)
        expected_extr_delay = Timing.rf_coinc_2_extr_delay(coinc_idx)

        self.dut._log.info(f"inj_check: Expected alignment count: {align_count}")
        self.dut._log.info(f"inj_check: Expected alignment term: {expected_align_term}")
        self.dut._log.info(
            f"inj_check: Expected coincidence term: {coinc_term}"
        )
        self.dut._log.info(f"inj_check: Expected injection delay: {expected_inj_delay}")
        self.dut._log.info(f"inj_check: Expected extraction delay: {expected_extr_delay}")
        self.dut._log.info(f"inj_check: Expected BR bucket: {expected_br_bucket}")

        return InjectionParameters(
            inj_delay = expected_inj_delay,
            extr_delay = expected_extr_delay,
            align_idx = align_count,
            coinc_idx = coinc_idx,
            br_bucket = expected_br_bucket,
        )

    async def injection_request(self, ar_bucket):
        self.dut._log.info(f"inj_req: AR bucket selection: {ar_bucket}")

        coinc_idx = Timing.arb_2_coinc_idx(ar_bucket)
        coinc_term = Timing.arb_2_coinc_term(ar_bucket)

        self.dut._log.info(f"inj_req: Coincidence index: {coinc_idx}")
        self.dut._log.info(f"inj_req: Coincidence term: {coinc_term}")

        target_val = ((coinc_term & 0xFFFF) << 16) | (coinc_idx & 0xFFFF)
        await self.write_target_csr(target_val)

        self.dut._log.info("inj_req: Programming Alignment CSR...")
        await self.write_align_csr((1 << 31))  # MSB=1 routes to alignCounterSel = 0
        await self.write_align_csr((1 << 30))  # Bit 30=1 routes to evgHeartbeatSel = 0

        self.dut._log.info("inj_req: Programming injection mode...")
        await self.write_csr((1 << 30) | 
            (int(self.dut.injectorSequenceControl.INJECTION_AR_MODE)))  # Bit 30 = 1 selects injection mode

        self.dut._log.info("inj_req: Triggering Injection Cycle...")
        await self.write_csr(0x80)

        self.dut._log.info("inj_req: Waiting for sequence start flag...")
        await RisingEdge(self.dut.evgSequenceStart)

        # Read back calculated/latched values
        await ClockCycles(self.dut.sysClk, 8)
        target_sta = await self.read_target_status()
        target_sta2 = await self.read_target_status2()
        inj_delay, extr_delay = await self.read_inj_delays()

        align_count = (target_sta2 & 0xFFFF0000) >> 16
        br_bucket = target_sta2 & 0xFFFF

        align_term = Timing.align_idx_2_align_term(align_count)

        self.dut._log.info(f"inj_req: Actual alignment count: {align_count}")
        self.dut._log.info(f"inj_req: Actual alignment term: {align_term}")
        self.dut._log.info(f"inj_req: Actual coincidence term: {coinc_term}")
        self.dut._log.info(f"inj_req: Actual injection delay: {inj_delay}")
        self.dut._log.info(f"inj_req: Actual extraction delay: {extr_delay}")
        self.dut._log.info(f"inj_req: Actual BR bucket: {br_bucket}")

        return InjectionParameters(
            inj_delay = inj_delay,
            extr_delay = extr_delay,
            align_idx = align_count,
            coinc_idx = coinc_idx,
            br_bucket = br_bucket,
        )

    async def injection_request_check(self, ar_bucket):
        # Start the request and the check task
        inj_check = cocotb.start_soon(
            with_timeout(self.injection_check_fsm(ar_bucket), 20, "ms")
        )
        inj_request = cocotb.start_soon(
            with_timeout(self.injection_request(ar_bucket), 20, "ms")
        )

        try:
            expected_params = await inj_check
        except SimTimeoutError:
            assert False, "FAIL: Wait for injection_check timeout"

        try:
            actual_params = await inj_request
        except SimTimeoutError:
            assert False, "FAIL: Wait for injection_request timeout"

        assert expected_params.align_idx == actual_params.align_idx, (
            f"FAIL: Expected alignment count ({expected_params.align_idx}) != "
            f"Actual alignment count ({actual_params.align_idx})"
        )

        assert expected_params.inj_delay == actual_params.inj_delay, (
            f"FAIL: Expected injection delay ({expected_params.inj_delay}) != "
            f"Actual injection delay ({actual_params.inj_delay})"
        )

        assert expected_params.extr_delay == actual_params.extr_delay, (
            f"FAIL: Expected extraction delay ({expected_params.extr_delay}) != "
            f"Actual extraction delay ({actual_params.extr_delay})"
        )

        assert expected_params.br_bucket == actual_params.br_bucket, (
            f"FAIL: Expected BR bucket ({expected_params.br_bucket}) != "
            f"Actual BR bucket ({actual_params.br_bucket})"
        )



async def do_randomized_tests(tb, num_tests=20):
    tb.dut._log.info(f"--- Starting {num_tests} Randomized Tests ---")

    for i in range(num_tests):
        tb.dut._log.info(f"Test #{i+1}...")
        num_cycles = await tb.wait_random()
        tb.dut._log.info(f"Random wait of {num_cycles} sysClk cycles")
        ar_bucket = tb.gen_random_ar_bucket()
        await tb.injection_request_check(ar_bucket=ar_bucket)

    tb.dut._log.info("--- Randomized Tests Complete ---\n")


async def do_coincidence_index_0_test(tb):
    tb.dut._log.info("--- Starting Coincidence Index 0 Test ---")

    num_cycles = await tb.wait_random()
    ar_bucket = 0
    await tb.injection_request_check(ar_bucket=ar_bucket)

    tb.dut._log.info("--- Coincidence Index 0 Test Complete ---\n")


@cocotb.test(timeout_time=10, timeout_unit="sec")
async def execute_all_tests(dut):
    tb = TB(dut)

    await tb.wait_sync()
    cocotb.start_soon(tb.monitor_sync())

    # Tests themselves
    await do_coincidence_index_0_test(tb)
    await do_randomized_tests(tb, num_tests=20)
