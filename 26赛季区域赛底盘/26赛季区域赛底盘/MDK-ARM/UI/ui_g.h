//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_line_t *ui_g_Ungroup_NewLine;

void ui_init_g_Ungroup(void);
void ui_update_g_Ungroup(void);
void ui_remove_g_Ungroup(void);

extern ui_interface_rect_t *ui_g_Ungroup1_L_track;
extern ui_interface_rect_t *ui_g_Ungroup1_R_track;
extern ui_interface_round_t *ui_g_Ungroup1_C_data_state;
extern ui_interface_rect_t *ui_g_Ungroup1_L_front_wheel;
extern ui_interface_rect_t *ui_g_Ungroup1_R_front_wheel;
extern ui_interface_rect_t *ui_g_Ungroup1_L_behind_wheel;
extern ui_interface_rect_t *ui_g_Ungroup1_R_behind_wheel;

void ui_init_g_Ungroup1(void);
void ui_update_g_Ungroup1(void);
void ui_remove_g_Ungroup1(void);

extern ui_interface_number_t *ui_g_Ungroup2_L_behind_state;
extern ui_interface_number_t *ui_g_Ungroup2_R_behind_state;
extern ui_interface_round_t *ui_g_Ungroup2_MY_Control_state;
extern ui_interface_number_t *ui_g_Ungroup2_L_track_state;
extern ui_interface_number_t *ui_g_Ungroup2_R_track_state;
extern ui_interface_number_t *ui_g_Ungroup2_L_front_state;
extern ui_interface_number_t *ui_g_Ungroup2_R_front_state;

void ui_init_g_Ungroup2(void);
void ui_update_g_Ungroup2(void);
void ui_remove_g_Ungroup2(void);

extern ui_interface_round_t *ui_g_Ungroup3_Big_Pitch;
extern ui_interface_rect_t *ui_g_Ungroup3_yaw;
extern ui_interface_round_t *ui_g_Ungroup3_smoll_pitch;
extern ui_interface_round_t *ui_g_Ungroup3_big_roll;
extern ui_interface_round_t *ui_g_Ungroup3_end_pitch;
extern ui_interface_round_t *ui_g_Ungroup3_end_roll;
extern ui_interface_round_t *ui_g_Ungroup3_claw_motor;

void ui_init_g_Ungroup3(void);
void ui_update_g_Ungroup3(void);
void ui_remove_g_Ungroup3(void);

extern ui_interface_number_t *ui_g_Ungroup4_big_pitch_state;
extern ui_interface_number_t *ui_g_Ungroup4_yaw_state;
extern ui_interface_number_t *ui_g_Ungroup4_smoll_pitch_state;
extern ui_interface_number_t *ui_g_Ungroup4_big_roll_state;
extern ui_interface_number_t *ui_g_Ungroup4_end_pitch_state;
extern ui_interface_number_t *ui_g_Ungroup4_end_roll_state;
extern ui_interface_number_t *ui_g_Ungroup4_claw_state;

void ui_init_g_Ungroup4(void);
void ui_update_g_Ungroup4(void);
void ui_remove_g_Ungroup4(void);

extern ui_interface_line_t *ui_g_Ungroup5_Big;
extern ui_interface_line_t *ui_g_Ungroup5_smoll;
extern ui_interface_line_t *ui_g_Ungroup5_end;
extern ui_interface_line_t *ui_g_Ungroup5_claw;
extern ui_interface_line_t *ui_g_Ungroup5_L;
extern ui_interface_line_t *ui_g_Ungroup5_R;
extern ui_interface_line_t *ui_g_Ungroup5_car;

void ui_init_g_Ungroup5(void);
void ui_update_g_Ungroup5(void);
void ui_remove_g_Ungroup5(void);


#endif // UI_g_H
