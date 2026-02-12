#ifndef APP_H
#define APP_H

#include <wx/wx.h>

namespace AFWS {

class Client : public wxApp {
public:
  virtual bool OnInit() override;
};

} // namespace AFWS

#endif
