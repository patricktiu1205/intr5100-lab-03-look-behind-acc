# -*- coding: utf-8 -*-
"""Experiment 3: Leader -> Student -> Following(IDM)."""
import sys
from PyQt6.QtWidgets import QApplication
from three_vehicle_experiment import ThreeVehicleExperimentGUI
from student_controller3 import student_control


class Experiment3GUI(ThreeVehicleExperimentGUI):
    experiment_name = "experiment3"
    window_title = "INTR5100 Experiment 3 - Leader -> Student -> IDM"
    roles = ("leader", "student", "following")
    idm_front_role = "student"
    idm_ego_role = "following"
    student_role = "student"
    student_front_role = "leader"

    def _student_args(self, states):
        return states["leader"], states["student"], states["following"]


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("INTR5100 Experiment 3")
    w = Experiment3GUI(student_control)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
