from arx_d_can.service_tools.change_damiao_id import (
    IdChangePlan,
    apply_id_change,
    build_id_change_plan,
    parse_id,
)
from arx_d_can.driver import CallError


def test_parse_id_accepts_decimal_and_hex():
    assert parse_id("7") == 7
    assert parse_id("0x07") == 7
    assert parse_id("0X17") == 0x17


def test_build_id_change_plan_defaults_feedback_ids_from_feedback_base():
    plan = build_id_change_plan(
        current_id=1,
        new_id=7,
        current_feedback_id=None,
        new_feedback_id=None,
        feedback_base=0x10,
    )

    assert plan == IdChangePlan(
        current_id=1,
        current_feedback_id=0x11,
        new_id=7,
        new_feedback_id=0x17,
    )


class FakeMotor:
    def __init__(self, events, motor_id, feedback_id):
        self._events = events
        self._motor_id = motor_id
        self._feedback_id = feedback_id

    def write_register_u32(self, rid, value):
        self._events.append(("write_u32", self._motor_id, self._feedback_id, rid, value))

    def store_parameters(self):
        self._events.append(("store", self._motor_id, self._feedback_id))

    def close(self):
        self._events.append(("motor_close", self._motor_id, self._feedback_id))


class FakeController:
    def __init__(self, events, fail_writes=None):
        self._events = events
        self._fail_writes = set(fail_writes or ())

    def add_damiao_motor(self, motor_id, feedback_id, model):
        self._events.append(("add", motor_id, feedback_id, model))
        return FakeMotorWithFailures(self._events, motor_id, feedback_id, self._fail_writes)

    def close_bus(self):
        self._events.append(("close_bus",))

    def close(self):
        self._events.append(("ctrl_close",))


class FakeMotorWithFailures(FakeMotor):
    def __init__(self, events, motor_id, feedback_id, fail_writes):
        super().__init__(events, motor_id, feedback_id)
        self._fail_writes = fail_writes

    def write_register_u32(self, rid, value):
        super().write_register_u32(rid, value)
        if (self._motor_id, self._feedback_id, rid, value) in self._fail_writes:
            raise CallError("simulated ack timeout")


def test_apply_id_change_reopens_after_each_id_switch():
    events = []
    plan = IdChangePlan(
        current_id=0x01,
        current_feedback_id=0x11,
        new_id=0x07,
        new_feedback_id=0x17,
    )

    apply_id_change(
        port="/dev/null",
        baud=1_000_000,
        model="4310",
        store=True,
        plan=plan,
        controller_factory=lambda _port, _baud: FakeController(events),
    )

    assert events == [
        ("add", 0x01, 0x11, "4310"),
        ("write_u32", 0x01, 0x11, 8, 0x07),
        ("motor_close", 0x01, 0x11),
        ("close_bus",),
        ("ctrl_close",),
        ("add", 0x07, 0x11, "4310"),
        ("write_u32", 0x07, 0x11, 7, 0x17),
        ("motor_close", 0x07, 0x11),
        ("close_bus",),
        ("ctrl_close",),
        ("add", 0x07, 0x17, "4310"),
        ("store", 0x07, 0x17),
        ("motor_close", 0x07, 0x17),
        ("close_bus",),
        ("ctrl_close",),
    ]


def test_apply_id_change_continues_after_id_write_ack_timeout():
    events = []
    plan = IdChangePlan(
        current_id=0x01,
        current_feedback_id=0x17,
        new_id=0x07,
        new_feedback_id=0x17,
    )

    apply_id_change(
        port="/dev/null",
        baud=1_000_000,
        model="4310",
        store=True,
        plan=plan,
        controller_factory=lambda _port, _baud: FakeController(
            events,
            fail_writes={(0x01, 0x17, 8, 0x07)},
        ),
    )

    assert ("store", 0x07, 0x17) in events
