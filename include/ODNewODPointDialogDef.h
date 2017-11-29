///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Feb 16 2016)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __ODNEWODPOINTDIALOGDEF_H__
#define __ODNEWODPOINTDIALOGDEF_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/intl.h>
#include <wx/string.h>
#include <wx/radiobox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/dialog.h>

#include "wxWTranslateCatalog.h"

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class ODNewODPointDialogDef
///////////////////////////////////////////////////////////////////////////////
class ODNewODPointDialogDef : public wxDialog 
{
	private:
	
	protected:
		wxRadioBox* m_radioBoxODPointType;
		wxButton* m_buttonOK;
		wxButton* m_buttonCancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnOK( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnCancel( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		ODNewODPointDialogDef( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = _("Point Type"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long style = wxCAPTION|wxSTAY_ON_TOP ); 
		~ODNewODPointDialogDef();
	
};

#endif //__ODNEWODPOINTDIALOGDEF_H__
