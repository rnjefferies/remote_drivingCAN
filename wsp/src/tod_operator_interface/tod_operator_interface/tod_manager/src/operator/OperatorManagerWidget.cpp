// Copyright 2020 Feiler

#include <algorithm>
#include "OperatorManagerWidget.h"
#include "ui_operatorWidget.h"
#include "IpAddressDeserializer.h"

QString backgroundGreen("background-color:rgb(154,205,50)");
QString backgroundRed("background-color:rgb(205,92,92)");

OperatorManagerWidget::OperatorManagerWidget(const std::string& pathToInitialIpAddressFile,
const std::string& searchedKey, QObject* operatorManager, QWidget *parent) :
    QMainWindow(parent),
    _ui(new Ui::OperatorManagerWidget) {
    _ui->setupUi(this);

    // register grouped buttons
    register_control_mode_buttons();
    register_input_device_buttons();
    register_video_rate_buttons();

    // enable only the steps that are possible in "idle" state
    control_buttons.enableButtons(false);
    input_device_buttons.enableButtons(true);
    video_rate_buttons.enableButtons(false);

    _ui->PushButton_Start->setEnabled(false);
    _ui->PushButton_Stop->setEnabled(false);
    _ui->PushButton_Disconnect->setEnabled(false);

    fillComboBox_IpAddressBroker_withIpAddr(pathToInitialIpAddressFile,
        searchedKey);
    readAndStoreOwnIpAddresses();
    addIpAddressesToComboBox();

    init_gui_status();
    _ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundRed);
    _ui->Label_LongReleased->setStyleSheet(backgroundRed);
    _ui->Label_LatReleased->setStyleSheet(backgroundRed);

    loadWidgetGeometryBeforeShow();
    this->show();
}

void OperatorManagerWidget::loadWidgetGeometryBeforeShow() {
    QSettings settings(QCoreApplication::organizationName(),
        QCoreApplication::applicationName());
    const QByteArray geometry = settings.value("geometry", QByteArray()).toByteArray();
    if ( !geometry.isEmpty() ) {
        restoreGeometry(geometry);
    }
}

void OperatorManagerWidget::fillComboBox_IpAddressBroker_withIpAddr(
        const std::string& pathToInitialIpAddressFile,
        const std::string& searchedKey) {
    std::vector<std::string> ipAddresses = IpAddressDeserializer::load(pathToInitialIpAddressFile, searchedKey);
    for ( auto ipAddress : ipAddresses ) {
        _ui->ComboBox_IpAddressBroker->addItem(QString::fromStdString(ipAddress));
    }
}

OperatorManagerWidget::~OperatorManagerWidget() {
    storeWidgetSettingsForReloadAtSamePosition();
    delete _ui;
}

void OperatorManagerWidget::storeWidgetSettingsForReloadAtSamePosition() {
    QSettings settings(QCoreApplication::organizationName(),
        QCoreApplication::applicationName());
    settings.setValue("geometry", saveGeometry());
}

void OperatorManagerWidget::init_gui_status() {
    gui_status.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    gui_status.vehicle_lat_approved = false;
    gui_status.vehicle_long_approved = false;
    gui_status.vehicle_emergency_stop_released = false;
    gui_status.vehicle_vehicle_ip_address = "127.0.0.1";

    gui_status.operator_control_mode =
        tod_msgs::Status::CONTROL_MODE_DIRECT;
    gui_status.operator_video_rate_mode =
        tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    gui_status.operator_input_device =
        InputDevice::FANATEC;
}

int OperatorManagerWidget::checkIpAddressValidity(const QHostAddress& newIpAddress) {
    return 0;
}

void OperatorManagerWidget::readAndStoreOwnIpAddresses() {
    // code copied from https://stackoverflow.com/questions/212528/get-the-ip-address-of-the-machine
    struct ifaddrs * ifAddrStruct = NULL;
    struct ifaddrs * ifa = NULL;
    void * tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            struct sockaddr_in * tempPointer = (struct sockaddr_in*)(ifa->ifa_addr);
            tmpAddrPtr = &(tempPointer->sin_addr);
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            // printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            std::string ip(addressBuffer);
            // store the ipv4 addresses
            _listOfIpAddressesOperator.push_back(ip);
        } else if (ifa->ifa_addr->sa_family == AF_INET6) { // check it is IP6
            // is a valid IP6 Address
            struct sockaddr_in6 * tempPointer = (struct sockaddr_in6*)(ifa->ifa_addr);
            tmpAddrPtr = &(tempPointer->sin6_addr);
            char addressBuffer[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
            // printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
        }
    }

    if (ifAddrStruct != NULL) freeifaddrs(ifAddrStruct);

    // sort my ip addresses:
    listWithPriorities.push_back(PRIO4);
    listWithPriorities.push_back(PRIO5);

    std::sort(_listOfIpAddressesOperator.begin(), _listOfIpAddressesOperator.end(), sortIpAddresses);
}

void OperatorManagerWidget::addIpAddressesToComboBox() {
    for ( auto it = _listOfIpAddressesOperator.begin();
        it != _listOfIpAddressesOperator.end(); ++it ) {
        _ui->ComboBox_IpAddressOperator->addItem((QString::fromLocal8Bit)((*it).c_str()));
    }
}

bool sortIpAddresses(const std::string& T1, const std::string& T2) {
    // if PRIO1 is found in T1 -> no work to do -> return true;
    // otherwise, sort is triggered with false
    for ( auto it = listWithPriorities.begin(); it != listWithPriorities.end(); ++it ) {
        size_t t1 = T1.find(*it);
        size_t t2 = T2.find(*it);
        if (t1 != std::string::npos) {
            return true;
        } else if ( t2 != std::string::npos ) {
            return false;
        }
    }
    return true;
}

void OperatorManagerWidget::on_PushButton_Connect_clicked() {
    change_button_status_after_clicked_on_connect();
    set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    emit signal_on_PushButton_Connect_clicked(
                _ui->ComboBox_IpAddressBroker->currentText().toStdString(),
                _ui->ComboBox_IpAddressOperator->currentText().toStdString());
}

void OperatorManagerWidget::change_button_status_after_clicked_on_connect() {
    // disables
    _ui->ComboBox_IpAddressBroker->setEnabled(false);
    _ui->ComboBox_IpAddressOperator->setEnabled(false);
    _ui->PushButton_Connect->setEnabled(false);

    // enables
    control_buttons.enableButtons(true);
    input_device_buttons.enableButtons(true);
    video_rate_buttons.enableButtons(true);

    _ui->PushButton_Start->setEnabled(true);
    _ui->PushButton_Disconnect->setEnabled(true);
}

void OperatorManagerWidget::on_PushButton_Start_clicked() {
    change_button_status_after_clicked_on_start();
    set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_TELEOPERATION);
    emit signal_on_PushButton_Start_clicked();
}

void OperatorManagerWidget::change_button_status_after_clicked_on_start() {
    // disables
    control_buttons.enableButtons(false);
    input_device_buttons.enableButtons(false);
    _ui->PushButton_Start->setEnabled(false);

    // enables
    _ui->PushButton_Stop->setEnabled(true);
}

void OperatorManagerWidget::on_PushButton_Stop_clicked() {
    change_button_status_after_clicked_on_stop();
    set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_UPLINK_ONLY);
    emit signal_on_PushButton_Stop_clicked();
}

void OperatorManagerWidget::change_button_status_after_clicked_on_stop() {
    // disables
    _ui->PushButton_Stop->setEnabled(false);

    // enables
    control_buttons.enableButtons(true);
    input_device_buttons.enableButtons(true);
    _ui->PushButton_Start->setEnabled(true);
}

void OperatorManagerWidget::on_PushButton_Disconnect_clicked() {
    change_button_status_after_clicked_on_disconnect();
    set_gui_connection_status_to(tod_msgs::Status::TOD_STATUS_IDLE);
    emit signal_on_PushButton_Disconnect_clicked();
}

void OperatorManagerWidget::change_button_status_after_clicked_on_disconnect() {
    // disables
    control_buttons.enableButtons(false);
    input_device_buttons.enableButtons(true);
    video_rate_buttons.enableButtons(false);

    _ui->PushButton_Start->setEnabled(false);
    _ui->PushButton_Stop->setEnabled(false);
    _ui->PushButton_Disconnect->setEnabled(false);

    // enables
    _ui->PushButton_Connect->setEnabled(true);
    _ui->ComboBox_IpAddressBroker->setEnabled(true);
    _ui->ComboBox_IpAddressOperator->setEnabled(true);
}

void OperatorManagerWidget::on_PushButton_DirectControl_clicked() {
    emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_DIRECT);
    control_buttons.switchFocusTo(_ui->PushButton_DirectControl, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_SharedControl_clicked() {
    emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_SHARED);
    control_buttons.switchFocusTo(_ui->PushButton_SharedControl, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_WayPointControl_clicked() {
    emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_WAYPOINT);
    control_buttons.switchFocusTo(_ui->PushButton_WayPointControl, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_PathGuidance_clicked() {
    emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_PATH_GUIDANCE);
    control_buttons.switchFocusTo(_ui->PushButton_PathGuidance, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_ObjectModification_clicked() {
    emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_PERCEPTION_MODIFICATION);
    control_buttons.switchFocusTo(_ui->PushButton_ObjectModification, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_SafeCorridorControl_clicked() {
    emit signal_control_mode_changed(tod_msgs::Status::CONTROL_MODE_SAFECORRIDOR);
    control_buttons.switchFocusTo(_ui->PushButton_SafeCorridorControl, backgroundGreen);
}

void OperatorManagerWidget::quitAll() {
    QApplication::quit();
}

void OperatorManagerWidget::init_control_mode_button_highlighting(const uint8_t control_mode) {
    switch (control_mode) {
        case tod_msgs::Status::CONTROL_MODE_DIRECT :
            control_buttons.switchFocusTo(_ui->PushButton_DirectControl, backgroundGreen);
            break;
        case tod_msgs::Status::CONTROL_MODE_SHARED :
            control_buttons.switchFocusTo(_ui->PushButton_SharedControl, backgroundGreen);
        break;
        case tod_msgs::Status::CONTROL_MODE_WAYPOINT :
            control_buttons.switchFocusTo(_ui->PushButton_WayPointControl, backgroundGreen);
        break;
        case tod_msgs::Status::CONTROL_MODE_PATH_GUIDANCE :
            control_buttons.switchFocusTo(_ui->PushButton_PathGuidance, backgroundGreen);
        break;
        case tod_msgs::Status::CONTROL_MODE_PERCEPTION_MODIFICATION :
            control_buttons.switchFocusTo(_ui->PushButton_ObjectModification, backgroundGreen);
        break;
        case tod_msgs::Status::CONTROL_MODE_SAFECORRIDOR :
            control_buttons.switchFocusTo(_ui->PushButton_SafeCorridorControl, backgroundGreen);
        break;
    }
}

void OperatorManagerWidget::init_video_rate_control_mode_button_highlighting(const uint8_t video_rate_control_mode) {
    switch (video_rate_control_mode) {
        case tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE :
            video_rate_buttons.switchFocusTo(_ui->PushButton_VideoRate_Single, backgroundGreen);
            break;
        case tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE :
            video_rate_buttons.switchFocusTo(_ui->PushButton_VideoRate_Collective, backgroundGreen);
            break;
        case tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_AUTOMATIC :
            video_rate_buttons.switchFocusTo(_ui->PushButton_VideoRate_PQOS, backgroundGreen);
            break;
    }
}

void OperatorManagerWidget::on_PushButton_BrowseAndOk_clicked() {
    std::string expectedPath = ros::package::getPath("tod_input_devices") + "/config";
    std::string fileName = QFileDialog::getOpenFileName(this,
        tr("Open Input Device Config Files"),
        QString::fromStdString(expectedPath), tr("Config Files (*.yaml)")).toStdString();
    _ui->LineEdit_PathToInputDevice->setText(QString::fromStdString(fileName));
    emit signal_input_device_changed(fileName);
}

void OperatorManagerWidget::on_PushButton_VideoRate_Single_clicked() {
    emit signal_video_rate_control_mode_changed(tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE);
    video_rate_buttons.switchFocusTo(_ui->PushButton_VideoRate_Single, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_VideoRate_Collective_clicked() {
    emit signal_video_rate_control_mode_changed(tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE);
    video_rate_buttons.switchFocusTo(_ui->PushButton_VideoRate_Collective, backgroundGreen);
}

void OperatorManagerWidget::on_PushButton_VideoRate_PQOS_clicked() {
    emit signal_video_rate_control_mode_changed(tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_AUTOMATIC);
    video_rate_buttons.switchFocusTo(_ui->PushButton_VideoRate_PQOS, backgroundGreen);
}

void OperatorManagerWidget::register_control_mode_buttons() {
    control_buttons.addWidget(_ui->PushButton_DirectControl);
    control_buttons.addWidget(_ui->PushButton_WayPointControl);
    control_buttons.addWidget(_ui->PushButton_ObjectModification);
    control_buttons.addWidget(_ui->PushButton_SharedControl);
    control_buttons.addWidget(_ui->PushButton_PathGuidance);
    control_buttons.addWidget(_ui->PushButton_SafeCorridorControl);
}

void OperatorManagerWidget::register_input_device_buttons() {
    input_device_buttons.addWidget(_ui->PushButton_BrowseAndOk);
    input_device_buttons.addWidget(_ui->LineEdit_PathToInputDevice);
}

void OperatorManagerWidget::register_video_rate_buttons() {
    video_rate_buttons.addWidget(_ui->PushButton_VideoRate_Single);
    video_rate_buttons.addWidget(_ui->PushButton_VideoRate_Collective);
    video_rate_buttons.addWidget(_ui->PushButton_VideoRate_PQOS);
}

bool OperatorManagerWidget::is_PushButton_SharedControl_enabled() {
    return _ui->PushButton_SharedControl->isEnabled();
}

void OperatorManagerWidget::set_ComboBox_IpAddressBroker_Text(
        const std::string& ip_addr) {
    _ui->ComboBox_IpAddressBroker->setCurrentText(QString::fromStdString(ip_addr));
}

void OperatorManagerWidget::set_ComboBox_IpAddressOperator_Text(
        const std::string& ip_addr) {
    _ui->ComboBox_IpAddressOperator->setCurrentText(QString::fromStdString(ip_addr));
}

tod_msgs::Status OperatorManagerWidget::get_gui_status() {
    return gui_status;
}

void OperatorManagerWidget::set_gui_connection_status_to(
            const uint8_t connection_status) {
    gui_status.tod_status = connection_status;
}

void OperatorManagerWidget::update_operator_tx_label(const unsigned int current_time) {
    _ui->LineEdit_LastTXOperator->setText(QString::number(current_time));
}

void OperatorManagerWidget::update_vehicle_tx_label(const unsigned int vehicle_time) {
    _ui->LineEdit_LastTXVehicle->setText(QString::number(vehicle_time));
}

void OperatorManagerWidget::set_gui_status_control_mode(const uint8_t control_mode) {
    gui_status.operator_control_mode = control_mode;
}

void OperatorManagerWidget::change_emergency_stop_released(bool released) {
    gui_status.vehicle_emergency_stop_released = released;
    update_safety_driver_status_labels();
}

void OperatorManagerWidget::lat_approved(bool approved) {
    gui_status.vehicle_lat_approved = approved;
    update_safety_driver_status_labels();
}

void OperatorManagerWidget::long_approved(bool approved) {
    gui_status.vehicle_long_approved = approved;
    update_safety_driver_status_labels();
}

void OperatorManagerWidget::update_safety_driver_status_labels() {
    if ( gui_status.vehicle_emergency_stop_released ) {
        _ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundGreen);
    } else {
        _ui->Label_EmergencyBreakReleased->setStyleSheet(backgroundRed);
    }

    if ( gui_status.vehicle_lat_approved ) {
        _ui->Label_LatReleased->setStyleSheet(backgroundGreen);
    } else {
        _ui->Label_LatReleased->setStyleSheet(backgroundRed);
    }

    if ( gui_status.vehicle_long_approved ) {
        _ui->Label_LongReleased->setStyleSheet(backgroundGreen);
    } else {
        _ui->Label_LongReleased->setStyleSheet(backgroundRed);
    }
}

void OperatorManagerWidget::set_nav_status(const std::string& nav_status) {
    gui_status.vehicle_nav_status = nav_status;
    _ui->LineEdit_Nav_Status->setText((QString::fromLocal8Bit)(nav_status.c_str()));
}

void OperatorManagerWidget::set_gps_pos_type(const std::string& gps_pos_type) {
    gui_status.vehicle_gps_pos_type = gps_pos_type;
    _ui->LineEdit_Pos_Type->setText((QString::fromLocal8Bit)(gps_pos_type.c_str()));
}
