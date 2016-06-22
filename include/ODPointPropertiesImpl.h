/***************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  OCPN Draw Point Properties Dialog support
 * Author:   Jon Gough
 *
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.         *
 **************************************************************************/
 
#ifndef __ODPointPropertiesImpl__
#define __ODPointPropertiesImpl__

/**
@file
Subclass of ODPointPropertiesDialog, which is generated by wxFormBuilder.
*/

#include "ODPointPropertiesDialog.h"
#include "ODPoint.h"
#include "TextPoint.h"
#include "BoundaryPoint.h"
#include <wx/bmpcbox.h>
#include <wx/fontdlg.h>

/*!
 * Control identifiers
 */

////@begin control identifiers
#define ID_ROUTEPROP 7000
#define SYMBOL_ROUTEPROP_TITLE wxT("Route Properties")
#define SYMBOL_ROUTEPROP_IDNAME ID_ROUTEPROP
#define SYMBOL_ROUTEPROP_SIZE wxSize(450, 300)
#define SYMBOL_ROUTEPROP_POSITION wxDefaultPosition

#ifdef __WXOSX__
#define SYMBOL_ROUTEPROP_STYLE wxCAPTION|wxRESIZE_BORDER|wxSYSTEM_MENU|wxCLOSE_BOX|wxSTAY_ON_TOP
#else
#define SYMBOL_ROUTEPROP_STYLE wxCAPTION|wxRESIZE_BORDER|wxSYSTEM_MENU|wxCLOSE_BOX
#endif


#define ID_TEXTCTRL            7001
#define ID_TEXTCTRL2           7002
#define ID_TEXTCTRL1           7003
#define ID_TEXTCTRL3           7005
#define ID_LISTCTRL            7004
#define ID_ROUTEPROP_CANCEL    7006
#define ID_ROUTEPROP_OK        7007
#define ID_ROUTEPROP_SPLIT     7107
#define ID_ROUTEPROP_EXTEND    7207
#define ID_ROUTEPROP_COPYTXT   7307
#define ID_ROUTEPROP_PRINT     7407
#define ID_WAYPOINTRANGERINGS  7507 
#define ID_SHOWWAYPOINTRANGERINGS  7607 
#define ID_PLANSPEEDCTL        7008
#define ID_TEXTCTRL4           7009
#define ID_TEXTCTRLDESC        7010
#define ID_STARTTIMECTL        7011
#define ID_TIMEZONESEL         7012
#define ID_TRACKLISTCTRL       7013
#define ID_RCLK_MENU_COPY_TEXT 7014
#define ID_RCLK_MENU_EDIT_WP   7015
#define ID_RCLK_MENU_DELETE    7016
#define ID_RCLK_MENU_COPY      7017
#define ID_RCLK_MENU_COPY_LL   7018
#define ID_RCLK_MENU_PASTE     7019
#define ID_RCLK_MENU_PASTE_LL  7020
#define ID_TIMEZONESEL_UTC     7021
#define ID_TIMEZONESEL_LOCAL   7022
#define ID_TIMEZONESEL_LMT     7023

#define ID_MARKPROP 8000
#define SYMBOL_MARKPROP_STYLE wxCAPTION|wxRESIZE_BORDER|wxSYSTEM_MENU|wxCLOSE_BOX
#define SYMBOL_MARKPROP_TITLE wxT("Waypoint Properties")
#define SYMBOL_MARKPROP_IDNAME ID_MARKPROP
#define SYMBOL_MARKPROP_SIZE wxSize(200, 300)
#define SYMBOL_MARKPROP_POSITION wxDefaultPosition
#define ID_MARKPROP_CANCEL 8001
#define ID_MARKPROP_OK 8002
#define ID_ICONCTRL 8003
#define ID_LATCTRL 8004
#define ID_LONCTRL 8005
#define ID_SHOWNAMECHECKBOX1 8006

////@end control identifiers

// class forward definitions
class ODIconCombo;

/** Implementing ODPointPropertiesDialog */
class ODPointPropertiesImpl : public ODPointPropertiesDialog
{
protected:
    void onRightClick( wxMouseEvent& event );
    void OnPositionCtlUpdated( wxCommandEvent& event );
    void OnArrivalRadiusChange( wxCommandEvent& event );
    void OnShowRangeRingsSelect( wxCommandEvent& event );
    void OnRangeRingsStepChange( wxCommandEvent& event );
    void OnDescChangedBasic( wxCommandEvent& event );
    void OnPointPropertiesOKClick( wxCommandEvent& event );
    void OnPointPropertiesCancelClick( wxCommandEvent& event );
    void OnPointPropertiesClose( wxCloseEvent& event );
    void OnComboboxSelected( wxCommandEvent& event );
    void OnCopyPasteLatLon( wxCommandEvent& event );
    void OnButtonClickFonts( wxCommandEvent& event );
    void OnRadioBoxPointType( wxCommandEvent& event );
    void SaveChanges();
    
    wxObject*               m_contextObject;
    wxSize                  m_defaultClientSize;
    ODIconCombo*            m_bODIComboBoxODPointIconName;

public:
    /** Constructor */
    ODPointPropertiesImpl( wxWindow* parent );
    virtual ~ODPointPropertiesImpl();
    void SetODPoint( ODPoint *pOP );
    ODPoint *GetODPoint( void ) { return (ODPoint *)m_pODPoint; }
    bool UpdateProperties( bool positionOnly = false );
    void SetDialogSize( void );
    void ValidateMark( void );
    
private:
      ODPoint       *m_pODPoint;
      TextPoint     *m_pTextPoint;
      BoundaryPoint *m_pBoundaryPoint;
      double        m_lat_save;
      double        m_lon_save;
      wxString      m_IconName_save;
      bool          m_bShowName_save;
      bool          m_bIsVisible_save;
      int           m_iFontNamePosition;
      wxFontDialog  *m_pfdDialog;
      double        m_dODPointRangeRingSteps;
      double        m_dODPointArrivalRadius;
};

#endif // __ODPointPropertiesImpl__
