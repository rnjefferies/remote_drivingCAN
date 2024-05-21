// Copyright 2020 Feiler

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QFileDialog>
#include <QtNetwork/QHostAddress>
#include <QtCore/QTimer>
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore>
#include <string>
#include <ros/package.h>
#include "tod_msgs/inputDevice.h"
#include "tod_msgs/Status.h"

#include <memory>
#include <map>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include "QtWidgetGroup.h"

namespace Ui {
class OperatorManagerWidget;
}

// sort function is used in order to provide the operator the most likely Ip Address of his computer
inline std::string PRIO4 { "127.0.0." };
inline std::string PRIO5 { "192.168." };
inline std::vector<std::string> listWithPriorities;

bool sortIpAddresses(const std::string& T1, const std::string& T2);

class OperatorManagerWidget : public QMainWindow {
    Q_OBJECT

public:
    explicit OperatorManagerWidget(const std::string& pathToInitialIpAddressFile,
        const std::string& searchedKey, QObject* operatorManager, QWidget *parent = 0);
    ~OperatorManagerWidget();

    // function needed for Gtester.cpp
    bool is_PushButton_SharedControl_enabled();
    void set_ComboBox_IpAddressBroker_Text(const std::string& ip_addr);
    void set_ComboBox_IpAddressOperator_Text(const std::string& ip_addr);
    void change_button_status_after_clicked_on_connect();
    void change_button_status_after_clicked_on_start();
    void change_button_status_after_clicked_on_stop();
    void change_button_status_after_clicked_on_disconnect();
    tod_msgs::Status get_gui_status();
    void set_gui_connection_status_to(const uint8_t connection_status);
    void set_gui_status_control_mode(const uint8_t control_mode);
    void update_safety_driver_status_labels();

    Ui::OperatorManagerWidget *_ui;
public slots:
    void init_control_mode_button_highlighting(const uint8_t control_mode);
    void init_video_rate_control_mode_button_highlighting(const uint8_t video_rate_control_mode);

    // necessary in order to close application from STRG + C (cli)
    void quitAll();

    void on_PushButton_Connect_clicked();
    void on_PushButton_Start_clicked();
    void on_PushButton_Stop_clicked();
    void on_PushButton_Disconnect_clicked();
    void on_PushButton_DirectControl_clicked();
    void on_PushButton_SharedControl_clicked();
    void on_PushButton_WayPointControl_clicked();
    void on_PushButton_PathGuidance_clicked();
    void on_PushButton_ObjectModification_clicked();
    void on_PushButton_SafeCorridorControl_clicked();
    void on_PushButton_BrowseAndOk_clicked();
    void on_PushButton_VideoRate_Single_clicked();
    void on_PushButton_VideoRate_Collective_clicked();
    void on_PushButton_VideoRate_PQOS_clicked();
    void update_operator_tx_label(const unsigned int current_time);
    void update_vehicle_tx_label(const unsigned int vehicle_time);
    void change_emergency_stop_released(bool released);
    void lat_approved(bool approved);
    void long_approved(bool approved);
    void set_nav_status(const std::string& nav_status);
    void set_gps_pos_type(const std::string& gps_pos_type);

private slots:

signals:
    void ipAddressesChanged(const QHostAddress& ipAddressOperator, const QHostAddress& ipAddressBroker);
    void signal_on_PushButton_Connect_clicked(const std::string& ip_addr_broker,
                                              const std::string& ip_addr_operator);
    void signal_on_PushButton_Disconnect_clicked();
    void signal_on_PushButton_Start_clicked();
    void signal_on_PushButton_Stop_clicked();
    void signal_control_mode_changed(const uint8_t control_mode);
    void signal_input_device_changed(const std::string& input_device);
    void signal_video_rate_control_mode_changed(const uint8_t video_rate_control_mode);

private:
    // store the user's input if valid
    QHostAddress _ipAddressBroker;
    QHostAddress _ipAddressOperator;

    // store the read Ip addresses of the operator
    std::vector<std::string> _listOfIpAddressesOperator;

    void fillComboBox_IpAddressBroker_withIpAddr(
        const std::string& pathToInitialIpAddressFile,
        const std::string& searchedKey);

    /* returns 1, if provided Ip Address does not have appropriate IPv4 or IPv6 format */
    int checkIpAddressValidity(const QHostAddress& userEnteredIpAddress);

    // read the host's ip addresses and sort them by calling sortIpAddresses()
    void readAndStoreOwnIpAddresses();

    // add ip addresses as items into combobox after being sorted
    void addIpAddressesToComboBox();

    QtWidgetGroup control_buttons;
    QtWidgetGroup input_device_buttons;
    QtWidgetGroup video_rate_buttons;
    void register_control_mode_buttons();
    void register_input_device_buttons();
    void register_video_rate_buttons();

    tod_msgs::Status gui_status;
    void init_gui_status();
    void loadWidgetGeometryBeforeShow();
    void storeWidgetSettingsForReloadAtSamePosition();
};

#endif // MAINWINDOW_H
