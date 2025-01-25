#include "wx/wx.h"
#include "wx/notebook.h"
#include <iostream>

using namespace std;

class MyApp : public wxApp {
public:
    virtual bool OnInit();
};

class MyFrame : public wxFrame {
public:
    MyFrame(const wxString& title);
    void OnClose(wxCloseEvent& event);
    void OnSetCablesTorque(wxCommandEvent& event);
    void OnControlCableMotor(wxCommandEvent& event);
    void OnControlLinearRailMotor(wxCommandEvent& event);
    void OnControlRobot(wxCommandEvent& event);
    void OnRequestCurrentPositionFromRPi(wxCommandEvent& event);
    void OnUpdateRobotPosFromFile(wxCommandEvent& event);
    void OnUpdateRobotConfigFromFile(wxCommandEvent& event);
    void OnClearException(wxCommandEvent& event);
};

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit() {
    MyFrame* frame = new MyFrame("Hello wxWidgets");
    frame->Show(true);
    return true;
}

MyFrame::MyFrame(const wxString& title) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(640, 580)) {
    wxPanel* panel = new wxPanel(this, wxID_ANY);

    // Create a vertical box sizer to manage layout
    wxBoxSizer* vBoxSizer = new wxBoxSizer(wxVERTICAL);

    // Create a horizontal box sizer for the main content
    wxBoxSizer* hBoxSizer = new wxBoxSizer(wxHORIZONTAL);

    // Create a notebook to hold the tabs
    wxNotebook* notebook = new wxNotebook(panel, wxID_ANY, wxDefaultPosition, wxSize(200, 400));

    // Create panels for each tab
    wxPanel* calibrationPanel = new wxPanel(notebook, wxID_ANY);
    wxPanel* operationPanel = new wxPanel(notebook, wxID_ANY);

    // Add a vertical box sizer to the calibration panel
    wxBoxSizer* calibrationVBoxSizer = new wxBoxSizer(wxVERTICAL);

    // Add the torqueSizer to the main calibrationVBoxSizer
    wxButton* setCablesTorqueButton = new wxButton(calibrationPanel, wxID_ANY, "Set Cables Torque");
    wxButton* controlCableMotorButton = new wxButton(calibrationPanel, wxID_ANY, "Control Cable Motor");
    wxButton* controlLinearRailMotorButton = new wxButton(calibrationPanel, wxID_ANY, "Control Linear Rail Motor");
    wxButton* controlRobotButton = new wxButton(calibrationPanel, wxID_ANY, "Control Robot");
    wxButton* requestCurrentPositionButton = new wxButton(calibrationPanel, wxID_ANY, "Request Current Position from RPi");
    wxButton* updateRobotPosFromFileButton = new wxButton(calibrationPanel, wxID_ANY, "Update Robot Pos from File");
    wxButton* updateRobotConfigFromFileButton = new wxButton(calibrationPanel, wxID_ANY, "Update Robot Config from File");
    wxButton* clearExceptionButton = new wxButton(calibrationPanel, wxID_ANY, "Clear Exception");
    
    
    calibrationVBoxSizer->Add(setCablesTorqueButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(controlCableMotorButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(controlLinearRailMotorButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(controlRobotButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(requestCurrentPositionButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(updateRobotPosFromFileButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(updateRobotConfigFromFileButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(clearExceptionButton, 1, wxEXPAND | wxALL, 5);

    // Set the sizer for the calibration panel
    calibrationPanel->SetSizer(calibrationVBoxSizer);

    // Add the panels to the notebook as tabs
    notebook->AddPage(calibrationPanel, "Calibration");
    notebook->AddPage(operationPanel, "Operation");

    // Add the notebook to the horizontal sizer
    hBoxSizer->Add(notebook, 1, wxEXPAND | wxALL, 5);

    // Create a panel for the right-hand side block
    wxPanel* rightPanel = new wxPanel(panel, wxID_ANY, wxDefaultPosition, wxSize(200, 400));
    rightPanel->SetBackgroundColour(*wxLIGHT_GREY); // Optional: set a background color

    // Create a vertical box sizer for the right block
    wxBoxSizer* rightVBoxSizer = new wxBoxSizer(wxVERTICAL);

    // Create a static box for the Cable Status with a title
    wxStaticBox* cableStaticBox = new wxStaticBox(rightPanel, wxID_ANY, "Cable Status");
    wxStaticBoxSizer* cableBoxSizer = new wxStaticBoxSizer(cableStaticBox, wxVERTICAL);

    // Create a panel for the Cable Status
    wxPanel* cableStatusPanel = new wxPanel(cableStaticBox, wxID_ANY, wxDefaultPosition, wxSize(200, 250));
    cableStatusPanel->SetBackgroundColour(*wxLIGHT_GREY); // Optional: set a background color

    // Create a flex grid sizer for cable details
    wxFlexGridSizer* cableGridSizer = new wxFlexGridSizer(8, 4, 5, 5); // 8 rows, 4 columns

    // Set flexible column sizes
    for (int i = 0; i < 4; ++i) {
        cableGridSizer->AddGrowableCol(i, 1); // Make the second column growable
    }

    // Add cable details
    for (int i = 0; i < 8; ++i) {
        wxString cableIndex = wxString::Format("%d", i + 1);
        wxString cableLength = wxString::Format("Length: %d", 100);
        wxString cableTorque = wxString::Format("Torque: %d", 50);
        wxString cableMotors = wxString::Format("Count: %d", 2);
        cableGridSizer->Add(new wxStaticText(cableStatusPanel, wxID_ANY, cableIndex), 0, wxALL, 5);
        cableGridSizer->Add(new wxStaticText(cableStatusPanel, wxID_ANY, cableLength), 0, wxALL, 5);
        cableGridSizer->Add(new wxStaticText(cableStatusPanel, wxID_ANY, cableMotors), 0, wxALL, 5);
        cableGridSizer->Add(new wxStaticText(cableStatusPanel, wxID_ANY, cableTorque), 0, wxALL, 5);
    }

    cableStatusPanel->SetSizer(cableGridSizer);
    cableBoxSizer->Add(cableStatusPanel, 1, wxEXPAND | wxALL, 5);
    rightVBoxSizer->Add(cableBoxSizer, 1, wxEXPAND | wxALL, 5);

    // Create a static box for the Robot Status with a title
    wxStaticBox* robotStaticBox = new wxStaticBox(rightPanel, wxID_ANY, "Robot Status");
    wxStaticBoxSizer* robotBoxSizer = new wxStaticBoxSizer(robotStaticBox, wxVERTICAL);

    // Create a panel for the Robot Status
    wxPanel* robotStatusPanel = new wxPanel(robotStaticBox, wxID_ANY, wxDefaultPosition, wxSize(200, 50));
    robotStatusPanel->SetBackgroundColour(*wxLIGHT_GREY); // Optional: set a background color

    // Create a flex grid sizer for robot status details
    wxFlexGridSizer* robotGridSizer = new wxFlexGridSizer(3, 2, 5, 5); // 3 rows, 2 columns, 5px gaps

    // Set flexible column sizes
    for (int i = 0; i < 2; ++i) {
        robotGridSizer->AddGrowableCol(i, 1);
    }
    // Add robot status details
    robotGridSizer->Add(new wxStaticText(robotStatusPanel, wxID_ANY, "X: 0.0"), 0, wxALL, 5);
    robotGridSizer->Add(new wxStaticText(robotStatusPanel, wxID_ANY, "Roll: 0.0"), 0, wxALL, 5);
    robotGridSizer->Add(new wxStaticText(robotStatusPanel, wxID_ANY, "Y: 0.0"), 0, wxALL, 5);
    robotGridSizer->Add(new wxStaticText(robotStatusPanel, wxID_ANY, "Pitch: 0.0"), 0, wxALL, 5);
    robotGridSizer->Add(new wxStaticText(robotStatusPanel, wxID_ANY, "Z: 0.0"), 0, wxALL, 5);
    robotGridSizer->Add(new wxStaticText(robotStatusPanel, wxID_ANY, "Yaw: 0.0"), 0, wxALL, 5);

    robotStatusPanel->SetSizer(robotGridSizer);
    robotBoxSizer->Add(robotStatusPanel, 1, wxEXPAND | wxALL, 5);
    rightVBoxSizer->Add(robotBoxSizer, 1, wxEXPAND | wxALL, 5);

    // Set the sizer for the right panel
    rightPanel->SetSizer(rightVBoxSizer);

    // Add the right panel to the horizontal sizer
    hBoxSizer->Add(rightPanel, 1, wxEXPAND | wxALL, 5);

    // Add the horizontal sizer to the vertical sizer
    vBoxSizer->Add(hBoxSizer, 1, wxEXPAND | wxALL, 5);

    // Create a text box for logging at the bottom
    wxTextCtrl* logTextBox = new wxTextCtrl(panel, wxID_ANY, "", wxDefaultPosition, wxSize(-1, 100), wxTE_MULTILINE | wxTE_READONLY);
    vBoxSizer->Add(logTextBox, 0, wxEXPAND | wxALL, 5);

    // Set the sizer for the main panel
    panel->SetSizer(vBoxSizer);

    // Bind the close event to a custom handler
    Bind(wxEVT_CLOSE_WINDOW, &MyFrame::OnClose, this);

    // Bind the button click event to a handler
    setCablesTorqueButton->Bind(wxEVT_BUTTON, &MyFrame::OnSetCablesTorque, this);
    controlCableMotorButton->Bind(wxEVT_BUTTON, &MyFrame::OnControlCableMotor, this);
    controlLinearRailMotorButton->Bind(wxEVT_BUTTON, &MyFrame::OnControlLinearRailMotor, this);
    controlRobotButton->Bind(wxEVT_BUTTON, &MyFrame::OnControlRobot, this);
    requestCurrentPositionButton->Bind(wxEVT_BUTTON, &MyFrame::OnRequestCurrentPositionFromRPi, this);
    updateRobotPosFromFileButton->Bind(wxEVT_BUTTON, &MyFrame::OnUpdateRobotPosFromFile, this);
    updateRobotConfigFromFileButton->Bind(wxEVT_BUTTON, &MyFrame::OnUpdateRobotConfigFromFile, this);
    clearExceptionButton->Bind(wxEVT_BUTTON, &MyFrame::OnClearException, this);

}

// Custom close event handler
void MyFrame::OnClose(wxCloseEvent& event) {
    Destroy(); // Close the application
    return;
    wxMessageDialog* confirmDialog = new wxMessageDialog(
        this, 
        "Are you sure you want to exit?", 
        "Confirm Exit", 
        wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION
    );

    if (confirmDialog->ShowModal() == wxID_YES) {
        Destroy(); // Close the application
    } else {
        event.Veto(); // Cancel the close event
    }
}


//Define the event handler for the Cable Torque Control
void MyFrame::OnSetCablesTorque(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Cable Torque Control", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Cable Motor Control
void MyFrame::OnControlCableMotor(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Cable Motor Control", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Linear Rail Motor Control
void MyFrame::OnControlLinearRailMotor(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Linear Rail Motor Control", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Robot Control
void MyFrame::OnControlRobot(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Robot Control", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Request Current Position from RPi
void MyFrame::OnRequestCurrentPositionFromRPi(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Request Current Position from RPi", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Update Robot Pos from File
void MyFrame::OnUpdateRobotPosFromFile(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Update Robot Pos from File", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Update Robot Config from File
void MyFrame::OnUpdateRobotConfigFromFile(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Update Robot Config from File", wxDefaultPosition, wxSize(300, 200));
    
    
}

//Define the event handler for the Clear Exception
void MyFrame::OnClearException(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Clear Exception", wxDefaultPosition, wxSize(300, 200));
    
}