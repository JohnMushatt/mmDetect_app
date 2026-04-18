import sys
import logging
from PySide6.QtWidgets import QApplication
from mmdetect.gui.main_window import MainWindow

def main():
    logging.basicConfig(level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    app = QApplication(sys.argv)
    app.setApplicationName("mmDetect")
    app.setOrganizationName("mmDetect")

    window = MainWindow()

    window.show()
    sys.exit(app.exec())



if __name__ == "__main__":
    main()