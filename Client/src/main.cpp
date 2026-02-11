#include <wx/wx.h>


class MyApp : public wxApp {
    public:
        bool OnInit() override;
};

wxIMPLEMENT_APP(MyApp);

class MyFrame : public wxFrame {
    public:
        MyFrame();
    private:
        void OnHello(wxCommandEvent& event);
        void OnExit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);

};

bool MyApp::OnInit() {
    MyFrame *frame = new MyFrame();
    frame->Show(true);
    return true;
}

enum {
    HelloID = 1
};

MyFrame::MyFrame() : wxFrame(nullptr, wxID_ANY, "HELLO WORLD") {
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(HelloID, "&Hello...\tCTRL+H", "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    SetMenuBar(menuBar);

    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");

    Bind(wxEVT_MENU, &MyFrame::OnHello, this, HelloID);
    Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);

}

void MyFrame::OnExit(wxCommandEvent& event) {
    Close(true);
}

void MyFrame::OnAbout(wxCommandEvent& event) {
    wxMessageBox("This is a wxWidgets Hello World example", "About Hello World", wxOK | wxICON_INFORMATION);
}

void MyFrame::OnHello(wxCommandEvent& event) {
    wxLogMessage("Hello world from wxWidgets!");
}