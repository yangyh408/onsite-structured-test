# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'TESS_API_EXAMPLE.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from PySide2.QtCore import Qt
from PySide2.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QMainWindow


class Ui_TESS_API_EXAMPLEClass(object):
    def setupUi(self, TESS_API_EXAMPLEClass):
        if not TESS_API_EXAMPLEClass.objectName():
            TESS_API_EXAMPLEClass.setObjectName(u"TESS_API_EXAMPLEClass")
        TESS_API_EXAMPLEClass.resize(262, 735)
        self.centralWidget = QWidget(TESS_API_EXAMPLEClass)
        self.centralWidget.setObjectName(u"centralWidget")
        self.verticalLayout_3 = QVBoxLayout(self.centralWidget)
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.btnOpenNet = QPushButton(self.centralWidget)
        self.btnOpenNet.setObjectName(u"btnOpenNet")

        self.showOnSite = QPushButton(self.centralWidget)
        self.showOnSite.setObjectName(u"showOnSite")
        self.verticalLayout_3.addWidget(self.showOnSite)

        self.showWayPoints = QPushButton(self.centralWidget)
        self.showWayPoints.setObjectName(u"showWayPoints")
        self.verticalLayout_3.addWidget(self.showWayPoints)

        self.verticalLayout_3.addWidget(self.btnOpenNet)

        self.btnStartSimu = QPushButton(self.centralWidget)
        self.btnStartSimu.setObjectName(u"btnStartSimu")

        self.verticalLayout_3.addWidget(self.btnStartSimu)

        self.btnPauseSimu = QPushButton(self.centralWidget)
        self.btnPauseSimu.setObjectName(u"btnPauseSimu")

        self.verticalLayout_3.addWidget(self.btnPauseSimu)

        self.btnStopSimu = QPushButton(self.centralWidget)
        self.btnStopSimu.setObjectName(u"btnStopSimu")

        self.verticalLayout_3.addWidget(self.btnStopSimu)

        self.groupBox = QGroupBox(self.centralWidget)
        self.groupBox.setObjectName(u"groupBox")
        self.verticalLayout_2 = QVBoxLayout(self.groupBox)
        self.verticalLayout_2.setSpacing(6)
        self.verticalLayout_2.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(1, -1, 1, -1)
        self.txtMessage = QTextBrowser(self.groupBox)
        self.txtMessage.setObjectName(u"txtMessage")

        self.verticalLayout_2.addWidget(self.txtMessage)

        self.verticalLayout_3.addWidget(self.groupBox)

        TESS_API_EXAMPLEClass.setCentralWidget(self.centralWidget)
        self.menuBar = QMenuBar(TESS_API_EXAMPLEClass)
        self.menuBar.setObjectName(u"menuBar")
        self.menuBar.setGeometry(QRect(0, 0, 262, 26))
        TESS_API_EXAMPLEClass.setMenuBar(self.menuBar)
        self.mainToolBar = QToolBar(TESS_API_EXAMPLEClass)
        self.mainToolBar.setObjectName(u"mainToolBar")
        TESS_API_EXAMPLEClass.addToolBar(Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QStatusBar(TESS_API_EXAMPLEClass)
        self.statusBar.setObjectName(u"statusBar")
        TESS_API_EXAMPLEClass.setStatusBar(self.statusBar)

        self.retranslateUi(TESS_API_EXAMPLEClass)

        QMetaObject.connectSlotsByName(TESS_API_EXAMPLEClass)

    # setupUi

    def retranslateUi(self, TESS_API_EXAMPLEClass):
        TESS_API_EXAMPLEClass.setWindowTitle(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"TESS_API_EXAMPLE", None))
        self.btnOpenNet.setText(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u6253\u5f00\u8def\u7f51", None))
        self.btnStartSimu.setText(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u542f\u52a8\u4eff\u771f", None))
        self.btnPauseSimu.setText(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u6682\u505c\u4eff\u771f", None))
        self.btnStopSimu.setText(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u505c\u6b62\u4eff\u771f", None))
        self.groupBox.setTitle(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u4fe1\u606f\u7a97", None))
        self.showOnSite.setText(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u004f\u006e\u0053\u0069\u0074\u0065", None))
        self.showWayPoints.setText(QCoreApplication.translate("TESS_API_EXAMPLEClass", u"\u663e\u793a\u8def\u5f84", None))
    # retranslateUi

