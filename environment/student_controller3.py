# -*- coding: utf-8 -*-
"""Student controller template for Experiment 3.

Vehicle order: Leader -> Student -> Following(IDM).
The student may use all three states. Replace only ``student_control``.
"""
from __future__ import annotations


def student_control(leader_state, student_state, following_state, history=None) -> float:
    # Minimal test controller for TA commissioning.
    v_cmd = 0.7 * float(leader_state.velocity) + 0.3 * float(following_state.velocity)
    return max(0.0, min(v_cmd, 100.0))
