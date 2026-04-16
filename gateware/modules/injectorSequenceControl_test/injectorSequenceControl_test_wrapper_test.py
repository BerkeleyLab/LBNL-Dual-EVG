import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, with_timeout
from cocotb.handle import Immediate
import random
import logging
import numpy as np


class Timing:
    @staticmethod
    def arb_2_coinc_idx(ar_bucket):
        return (5 * ar_bucket) % 304

    @staticmethod
    def arb_2_coinc_term(ar_bucket):
        coinc_idx = Timing.arb_2_coinc_idx(ar_bucket)
        return (43 * coinc_idx) % 125

    @staticmethod
    def align_idx_2_align_term(br_ar_align):
        return (72 * br_ar_align) % 125

    @staticmethod
    def terms_2_br_bucket(coinc_term, align_term):
        return (coinc_term + align_term) % 125

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

        if (self.dut.clkGenSynceds.value.is_resolvable and
            self.dut.clkGenSynceds.value.to_unsigned() == expected_val):
            return True
        else:
            return False

    async def wait_sync(self):
        await self.wait_for_heartbeat()
        await self.wait_for_heartbeat()
        await RisingEdge(self.dut.evgTxClk)
        await RisingEdge(self.dut.evgTxClk)

        assert self.is_clk_gen_synched(), \
                f"Clock generation is not synchronized"

    async def monitor_sync(self):
        while True:
            await RisingEdge(self.dut.evgTxClk)

            assert self.is_clk_gen_synched(), \
                    f"Clock generation is not synchronized"

    async def _write_csr(self, strobe_name, value):
        stb = getattr(self.dut, strobe_name);

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
        reg = getattr(self.dut, reg_name);

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

    async def read_rf_coinc_count(self):
        await RisingEdge(self.dut.evgTxClk)
        return self.dut.evgRFCoincCountMon.value.to_unsigned()

    async def read_rf_align_count(self):
        await RisingEdge(self.dut.evgTxClk)
        return self.dut.evgRFAlignCountMon.value.to_unsigned()

    def gen_random_ar_bucket(self):
        ar_bucket = random.randint(0, 303)
        return ar_bucket

    async def wait_random(self):
        num_cycles = random.randint(0, 1000)
        await ClockCycles(self.dut.sysClk, num_cycles)
        return num_cycles

    async def injection_request(self, ar_bucket):
        self.dut._log.info(f"AR bucket selection: {ar_bucket}")

        coinc_idx = Timing.arb_2_coinc_idx(ar_bucket)
        coinc_term = Timing.arb_2_coinc_term(ar_bucket)

        self.dut._log.info(f"Coincidence index: {coinc_idx}")
        self.dut._log.info(f"Coincidence term: {coinc_term}")

        target_val = ((coinc_term & 0xFFFF) << 16) | (coinc_idx & 0xFFFF)
        await self.write_target_csr(target_val)

        self.dut._log.info("Programming Alignment CSR...")
        await self.write_align_csr((1 << 31))  # MSB=1 routes to alignCounterSel = 0
        await self.write_align_csr((1 << 30))  # Bit 30=1 routes to evgHeartbeatSel = 0

        self.dut._log.info("Triggering Injection Cycle...")
        await self.write_csr(0x80)

        self.dut._log.info("Waiting for sequence start flag...")
        try:
            await with_timeout(RisingEdge(self.dut.evgSequenceStart), 2.5, 'ms')
        except SimTimeoutError:
            assert False, "FAIL: Timeout waiting for evgSequenceStart assertion"

        align_count = await self.read_rf_align_count()
        coinc_count = await self.read_rf_coinc_count()

        self.dut._log.info(f"Alignment count: {align_count}")
        self.dut._log.info(f"Coincidence count: {coinc_count}")

        align_term = Timing.align_idx_2_align_term(align_count)
        expected_br_bucket = Timing.terms_2_br_bucket(coinc_term, align_term)

        self.dut._log.info(f"Alignment term: {align_term}")
        self.dut._log.info(f"Expected BR bucket: {expected_br_bucket}")

        await ClockCycles(self.dut.sysClk, 8)
        actual_br_bucket = await self.read_target_status2()
        self.dut._log.info(f"Actual BR bucket value: {actual_br_bucket}")

        assert expected_br_bucket == actual_br_bucket, \
                f"FAIL: Expected BR bucket ({expected_br_bucket}) != " \
                f"Acutal BR bucket ({actual_br_bucket})"


@cocotb.test(timeout_time=10, timeout_unit='sec')
async def test(dut, length=1):
    tb = TB(dut)

    await tb.wait_sync()
    cocotb.start_soon(tb.monitor_sync())

    for i in range(20):
        tb.dut._log.info(f"Test #{i+1}...")
        num_cycles = await tb.wait_random()
        tb.dut._log.info(f"Random wait of {num_cycles} sysClk cycles")
        ar_bucket = tb.gen_random_ar_bucket()
        await tb.injection_request(ar_bucket = ar_bucket)
