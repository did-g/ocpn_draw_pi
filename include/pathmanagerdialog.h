/*
    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor,
    Boston, MA  02110-1301, USA.

    ---
    Copyright (C) 2010, Anders Lund <anders@alweb.dk>
 */

#ifndef _PathManagerDialog_h_
#define _PathManagerDialog_h_

#include <wx/dialog.h>
#include <wx/timer.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/listctrl.h>
#include <wx/notebook.h>

enum {
      SORT_ON_DISTANCE  = 1,
      SORT_ON_NAME
};

enum TrackContextMenu {
      TRACK_MERGE  = 1,
      TRACK_COPY_TEXT,
      TRACK_CLEAN
};

class wxButton;
class Path;
class OCPNPoint;
class Layer;

class PathManagerDialog : public wxDialog {
      DECLARE_EVENT_TABLE()

      public:
            PathManagerDialog(wxWindow *parent);
            ~PathManagerDialog();
            void SetColorScheme();
            void UpdatePathListCtrl();     // Rebuild route list
            void UpdateOCPNPointsListCtrl(OCPNPoint *rp_select = NULL, bool b_retain_sort = false);
            void UpdateLayListCtrl();
            void UpdateOCPNPointsListCtrlViz();
            
            void OnTabSwitch(wxNotebookEvent& event);
            static void OCPNPointShowPropertiesDialog( OCPNPoint* wp, wxWindow* parent );
            void ShowPathPropertiesDialog ( Path *path );
//            void TrackToRoute( Track *track );

      private:
            void Create();
            void MakeAllPathsInvisible();  // Mark all boundaries as invisible. Does not flush settings.
            void ZoomtoPath(Path *path); // Attempt to zoom path into the view
            void UpdatePathButtons();
            void UpdateOCPNPointButtons();
            void UpdateLayButtons();           // Correct button state
            
            void ToggleLayerContentsOnChart(Layer *layer);
            void ToggleLayerContentsOnListing(Layer *layer);
            void ToggleLayerContentsNames(Layer *layer);

            // event handlers
            void OnPathSelected(wxListEvent &event);
            void OnPathDefaultAction(wxListEvent &event);
            void OnPathToggleVisibility(wxMouseEvent &event);
            void OnPathColumnClicked(wxListEvent &event);
            void OnPathPropertiesClick(wxCommandEvent &event);
            void OnPathBtnLeftDown(wxMouseEvent &event); // record control key state for some action buttons
            void OnPathZoomtoClick(wxCommandEvent &event);
            void OnPathDeleteClick(wxCommandEvent &event);
            void OnPathExportClick(wxCommandEvent &event);
            void OnPathDeleteAllClick(wxCommandEvent &event);
            void OnPathActivateClick(wxCommandEvent &event);
            void OnTrkDefaultAction(wxListEvent &event);
            void OnOCPNPointDefaultAction(wxListEvent &event);
            void OnOCPNPointNewClick(wxCommandEvent &event);
            void OnOCPNPointPropertiesClick(wxCommandEvent &event);
            void OnOCPNPointZoomtoClick(wxCommandEvent &event);
            void OnOCPNPointDeleteClick(wxCommandEvent &event);
//            void OnOCPNPointGoToClick(wxCommandEvent &event);
            void OnOCPNPointExportClick(wxCommandEvent &event);
//            void OnOCPNPointSendToGPSClick(wxCommandEvent &event);
            void OnOCPNPointDeleteAllClick(wxCommandEvent &event);
            void OnOCPNPointSelected(wxListEvent &event);
            void OnOCPNPointToggleVisibility(wxMouseEvent &event);
            void OnOCPNPointColumnClicked(wxListEvent &event);

            void OnLayDefaultAction(wxListEvent &event);
            void OnLayNewClick(wxCommandEvent &event);
            void OnLayPropertiesClick(wxCommandEvent &event);
            void OnLayToggleChartClick(wxCommandEvent &event);
            void OnLayToggleListingClick(wxCommandEvent &event);
            void OnLayToggleNamesClick(wxCommandEvent &event);
            void OnLayDeleteClick(wxCommandEvent &event);
            void OnLaySelected(wxListEvent &event);
            void OnLayToggleVisibility(wxMouseEvent &event);
            void OnLayColumnClicked(wxListEvent &event);

            void OnImportClick(wxCommandEvent &event);
            void OnExportClick(wxCommandEvent &event);
            void OnExportVizClick(wxCommandEvent &event);
            
            // properties
            wxNotebook *m_pNotebook;
            wxPanel    *m_pPanelOCPNPoint;
            wxPanel    *m_pPanelPath;
            wxPanel     *m_pPanelLay;
            wxListCtrl *m_pPathListCtrl;
            wxListCtrl *m_pOCPNPointListCtrl;
            wxListCtrl  *m_pLayListCtrl;
            

            wxButton *btnPathDelete;
            wxButton *btnPathExport;
            wxButton *btnPathZoomto;
            wxButton *btnPathProperties;
            wxButton *btnPathDeleteAll;
            wxButton *btnPathActivate;
            
            wxButton *btnOCPNPointNew;
            wxButton *btnOCPNPointProperties;
            wxButton *btnOCPNPointZoomto;
            wxButton *btnOCPNPointDelete;
            wxButton *btnOCPNPointGoTo;
            wxButton *btnOCPNPointExport;
            wxButton *btnOCPNPointSendToGPS;
            wxButton *btnOCPNPointDeleteAll;
            
            wxButton *btnLayNew;
            //wxButton *btnLayProperties;
            wxButton *btnLayToggleChart;
            wxButton *btnLayToggleListing;
            wxButton *btnLayToggleNames;
            wxButton *btnLayDelete;
            
            wxButton *btnImport;
            wxButton *btnExport;
            wxButton *btnExportViz;
            
            bool m_bPossibleClick;    // do
            bool m_bCtrlDown;         // record control key state for some action buttons
            bool m_bNeedConfigFlush;  // if true, update config in destructor

            int m_lastOCPNPointItem;
            int m_lastPathItem;
};

#endif // _PathManagerDialog_h_
// kate: indent-width 6; indent-mode cstyle; space-indent on;