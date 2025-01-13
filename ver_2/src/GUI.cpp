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
    void OnControlCableMotor(wxCommandEvent& event);
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
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Set Cables Torque"), 1, wxEXPAND | wxALL, 5);
    wxButton* controlCableMotorButton = new wxButton(calibrationPanel, wxID_ANY, "Control Cable Motor");
    calibrationVBoxSizer->Add(controlCableMotorButton, 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Control Linear Rail Motor"), 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Control Robot"), 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Request Current Position from RPi"), 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Update Robot Pos from File"), 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Update Robot Config from File"), 1, wxEXPAND | wxALL, 5);
    calibrationVBoxSizer->Add(new wxButton(calibrationPanel, wxID_ANY, "Clear Exception"), 1, wxEXPAND | wxALL, 5);

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
    controlCableMotorButton->Bind(wxEVT_BUTTON, &MyFrame::OnControlCableMotor, this);
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

// Define the event handler in the MyFrame class
void MyFrame::OnControlCableMotor(wxCommandEvent& event) {
    // Create a new frame or dialog
    wxFrame* newWindow = new wxFrame(this, wxID_ANY, "Cable Motor Control", wxDefaultPosition, wxSize(300, 200));
    
    // Optionally, add content to the new window
    wxPanel* panel = new wxPanel(newWindow, wxID_ANY);
    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(new wxStaticText(panel, wxID_ANY, "Cable Motor Control Options"), 0, wxALL, 10);
    panel->SetSizer(sizer);

    // Show the new window
    newWindow->Show();
}
