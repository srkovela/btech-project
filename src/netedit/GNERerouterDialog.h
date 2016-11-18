/****************************************************************************/
/// @file    GNERerouterDialog.h
/// @author  Pablo Alvarez Lopez
/// @date    April 2016
/// @version $Id: GNERerouterDialog.h 21885 2016-11-03 13:20:39Z palcraft $
///
/// Dialog for edit rerouters
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef GNERerouterDialog_h
#define GNERerouterDialog_h

// ===========================================================================
// included modules
// ===========================================================================

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNEAdditionalDialog.h"


// ===========================================================================
// class declarations
// ===========================================================================

class GNERerouter;

// ===========================================================================
// class definitions
// ===========================================================================

/**
 * @class GNERerouterDialog
 * @brief Dialog for edit rerouters
 */

class GNERerouterDialog : public GNEAdditionalDialog {
    /// @brief FOX-declaration
    FXDECLARE(GNERerouterDialog)

public:
    // Constructor
    GNERerouterDialog(GNERerouter* rerouterParent);

    // destructor
    ~GNERerouterDialog();

    /// @name FOX-callbacks
    /// @{
    /// @brief event after press accept button
    long onCmdAccept(FXObject*, FXSelector, void*);

    /// @brief event after press cancel button
    long onCmdCancel(FXObject*, FXSelector, void*);

    /// @brief event after press reset button
    long onCmdReset(FXObject*, FXSelector, void*);
    /// @}

protected:
    /// @brief FOX needs this
    GNERerouterDialog() {}

    /// @brief pointer to rerouter parent
    GNERerouter* myRerouterParent;

private:
    /// @brief update data table
    void updateTable();

    /// @brief Invalidated copy constructor.
    GNERerouterDialog(const GNERerouterDialog&);

    /// @brief Invalidated assignment operator.
    GNERerouterDialog& operator=(const GNERerouterDialog&);
};

#endif
