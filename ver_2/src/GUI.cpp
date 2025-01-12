#include "wx/wx.h"
#include <iostream>

using namespace std;

class MyApp : public wxApp {
public:
    virtual bool OnInit();
};

class MyFrame : public wxFrame {
public:
    MyFrame(const wxString& title);
};

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit() {
    MyFrame* frame = new MyFrame("Hello wxWidgets");
    frame->Show(true);
    return true;
}

MyFrame::MyFrame(const wxString& title) : wxFrame(NULL, wxID_ANY, title) {
    wxPanel* panel = new wxPanel(this, wxID_ANY);
    wxButton* button = new wxButton(panel, wxID_EXIT, "Close", wxPoint(20, 20));
    Bind(wxEVT_BUTTON, [](wxCommandEvent&) { wxExit(); }, wxID_EXIT);
}
