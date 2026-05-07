import sys
from PyQt5.QtWidgets import QApplication, QLabel
app = QApplication(sys.argv)
label = QLabel("PyQt5 安装成功！")
label.show()
sys.exit(app.exec_())