from enum import Enum


class DualMpcState(str, Enum):
    WAIT_TOPICS = "WAIT_TOPICS"
    PRESTREAM = "PRESTREAM"
    SET_OFFBOARD = "SET_OFFBOARD"
    ARM = "ARM"
    ALIGN_INITIAL_STATE = "ALIGN_INITIAL_STATE"
    TRACK = "TRACK"
    TERMINAL_HOLD = "TERMINAL_HOLD"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class DualMpcStateMachine:
    def __init__(self):
        self.state = DualMpcState.WAIT_TOPICS

    def transition(self, state: DualMpcState) -> None:
        self.state = DualMpcState(state)
