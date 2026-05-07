//--------------------------------------------------------------------------------//
//-                   			kb_utils.h                                           -//
//                                                                               -//
//-  Copyright (C) Julien Tharin, K-Team S.A. 2011                               -//
//-  This library is free software; you can redistribute it and/or               -//
//-  modify it under the terms of the GNU Lesser General Public                  -//
//-  License as published by the Free Software Foundation; either                -//
//-  version 2.1 of the License, or any later version.                           -//
//-                                                                              -//
//-  This library is distributed in the hope that it will be useful,             -//
//-  but WITHOUT ANY WARRANTY; without even the implied warranty of              -//
//-  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU           -//
//-  Lesser General Public License for more details.                             -//
//-                                                                              -//
//-  You should have received a copy of the GNU Lesser General Public            -//
//-  License along with this library; if not, write to the Free Software         -//
//-  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA   -//
//-                                                                              -//
//-                               __  __  ________                               -//
//- K-Team S.A.                  |  |/  /|__    __|___  _____  ___  ___          -//
//- Rue Galilee 9. Y-Park,       |     / __ |  | _____|/  _  \|   \/   |         -//
//- 1400 Yverdon-les-Bains       |  |  \    |  | ____|/  /_\  |        |         -//
//- Switzerland                  |__|\__\   |__|______|_/   \_|__|\/|__|         -//
//- jtharin@k-team.com   tel:+41 24 423 89 75 fax:+41 24 423 8960                -//
//-                                                                              -//
//--------------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
/*!   \file kb_utils.h
      \brief Header of Useful functions of libkhepera
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef __kb_utils__
#define __kb_utils__

/* ---- Constants and Types ---------------------------------------------- */


/* ---- Function Prototypes ---------------------------------------------- */

void kb_change_term_mode(int dir);

int kb_kbhit(void);

void kb_clrscr(void);

void kb_move_cursor(int c, int l);

void kb_move_cursor_column(int c);

void kb_move_cursor_line(int l);

void kb_erase_line(int line);


#endif // #ifndef __kb_utils__

