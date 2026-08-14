//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_1_frame_t ui_g_Ungroup_0;

ui_interface_line_t *ui_g_Ungroup_NewLine = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[0]);

void _ui_init_g_Ungroup_0(void) {
    for (int i = 0; i < 1; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_proc_1_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0(void) {
    for (int i = 0; i < 1; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }
		
    ui_proc_1_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0(void) {
    for (int i = 0; i < 1; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}


void ui_init_g_Ungroup(void) {
    _ui_init_g_Ungroup_0();
}

void ui_update_g_Ungroup(void) {
    _ui_update_g_Ungroup_0();
}

void ui_remove_g_Ungroup(void) {
    _ui_remove_g_Ungroup_0();
}

ui_7_frame_t ui_g_Ungroup1_0;

ui_interface_rect_t *ui_g_Ungroup1_L_track = (ui_interface_rect_t*)&(ui_g_Ungroup1_0.data[0]);
ui_interface_rect_t *ui_g_Ungroup1_R_track = (ui_interface_rect_t*)&(ui_g_Ungroup1_0.data[1]);
ui_interface_round_t *ui_g_Ungroup1_C_data_state = (ui_interface_round_t*)&(ui_g_Ungroup1_0.data[2]);
ui_interface_rect_t *ui_g_Ungroup1_L_front_wheel = (ui_interface_rect_t*)&(ui_g_Ungroup1_0.data[3]);
ui_interface_rect_t *ui_g_Ungroup1_R_front_wheel = (ui_interface_rect_t*)&(ui_g_Ungroup1_0.data[4]);
ui_interface_rect_t *ui_g_Ungroup1_L_behind_wheel = (ui_interface_rect_t*)&(ui_g_Ungroup1_0.data[5]);
ui_interface_rect_t *ui_g_Ungroup1_R_behind_wheel = (ui_interface_rect_t*)&(ui_g_Ungroup1_0.data[6]);

void _ui_init_g_Ungroup1_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup1_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup1_0.data[i].figure_name[1] = 1;
        ui_g_Ungroup1_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup1_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Ungroup1_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup1_L_track->figure_type = 1;
    ui_g_Ungroup1_L_track->operate_type = 1;
    ui_g_Ungroup1_L_track->layer = 0;
    ui_g_Ungroup1_L_track->color = 6;
    ui_g_Ungroup1_L_track->start_x = 25;
    ui_g_Ungroup1_L_track->start_y = 790;
    ui_g_Ungroup1_L_track->width = 3;
    ui_g_Ungroup1_L_track->end_x = 165;
    ui_g_Ungroup1_L_track->end_y = 870;

    ui_g_Ungroup1_R_track->figure_type = 1;
    ui_g_Ungroup1_R_track->operate_type = 1;
    ui_g_Ungroup1_R_track->layer = 0;
    ui_g_Ungroup1_R_track->color = 6;
    ui_g_Ungroup1_R_track->start_x = 180;
    ui_g_Ungroup1_R_track->start_y = 790;
    ui_g_Ungroup1_R_track->width = 3;
    ui_g_Ungroup1_R_track->end_x = 320;
    ui_g_Ungroup1_R_track->end_y = 870;

    ui_g_Ungroup1_C_data_state->figure_type = 2;
    ui_g_Ungroup1_C_data_state->operate_type = 1;
    ui_g_Ungroup1_C_data_state->layer = 0;
    ui_g_Ungroup1_C_data_state->color = 0;
    ui_g_Ungroup1_C_data_state->start_x = 95;
    ui_g_Ungroup1_C_data_state->start_y = 520;
    ui_g_Ungroup1_C_data_state->width = 10;
    ui_g_Ungroup1_C_data_state->r = 20;

    ui_g_Ungroup1_L_front_wheel->figure_type = 1;
    ui_g_Ungroup1_L_front_wheel->operate_type = 1;
    ui_g_Ungroup1_L_front_wheel->layer = 0;
    ui_g_Ungroup1_L_front_wheel->color = 4;
    ui_g_Ungroup1_L_front_wheel->start_x = 55;
    ui_g_Ungroup1_L_front_wheel->start_y = 675;
    ui_g_Ungroup1_L_front_wheel->width = 3;
    ui_g_Ungroup1_L_front_wheel->end_x = 140;
    ui_g_Ungroup1_L_front_wheel->end_y = 780;

    ui_g_Ungroup1_R_front_wheel->figure_type = 1;
    ui_g_Ungroup1_R_front_wheel->operate_type = 1;
    ui_g_Ungroup1_R_front_wheel->layer = 0;
    ui_g_Ungroup1_R_front_wheel->color = 4;
    ui_g_Ungroup1_R_front_wheel->start_x = 205;
    ui_g_Ungroup1_R_front_wheel->start_y = 675;
    ui_g_Ungroup1_R_front_wheel->width = 3;
    ui_g_Ungroup1_R_front_wheel->end_x = 290;
    ui_g_Ungroup1_R_front_wheel->end_y = 780;

    ui_g_Ungroup1_L_behind_wheel->figure_type = 1;
    ui_g_Ungroup1_L_behind_wheel->operate_type = 1;
    ui_g_Ungroup1_L_behind_wheel->layer = 0;
    ui_g_Ungroup1_L_behind_wheel->color = 4;
    ui_g_Ungroup1_L_behind_wheel->start_x = 55;
    ui_g_Ungroup1_L_behind_wheel->start_y = 560;
    ui_g_Ungroup1_L_behind_wheel->width = 3;
    ui_g_Ungroup1_L_behind_wheel->end_x = 140;
    ui_g_Ungroup1_L_behind_wheel->end_y = 665;

    ui_g_Ungroup1_R_behind_wheel->figure_type = 1;
    ui_g_Ungroup1_R_behind_wheel->operate_type = 1;
    ui_g_Ungroup1_R_behind_wheel->layer = 0;
    ui_g_Ungroup1_R_behind_wheel->color = 4;
    ui_g_Ungroup1_R_behind_wheel->start_x = 205;
    ui_g_Ungroup1_R_behind_wheel->start_y = 560;
    ui_g_Ungroup1_R_behind_wheel->width = 3;
    ui_g_Ungroup1_R_behind_wheel->end_x = 290;
    ui_g_Ungroup1_R_behind_wheel->end_y = 665;


    ui_proc_7_frame(&ui_g_Ungroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup1_0, sizeof(ui_g_Ungroup1_0));
}

void _ui_update_g_Ungroup1_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup1_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Ungroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup1_0, sizeof(ui_g_Ungroup1_0));
}

void _ui_remove_g_Ungroup1_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup1_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Ungroup1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup1_0, sizeof(ui_g_Ungroup1_0));
}


void ui_init_g_Ungroup1(void) {
    _ui_init_g_Ungroup1_0();
}

void ui_update_g_Ungroup1(void) {
    _ui_update_g_Ungroup1_0();
}

void ui_remove_g_Ungroup1(void) {
    _ui_remove_g_Ungroup1_0();
}

ui_7_frame_t ui_g_Ungroup2_0;

ui_interface_number_t *ui_g_Ungroup2_L_behind_state = (ui_interface_number_t*)&(ui_g_Ungroup2_0.data[0]);
ui_interface_number_t *ui_g_Ungroup2_R_behind_state = (ui_interface_number_t*)&(ui_g_Ungroup2_0.data[1]);
ui_interface_round_t *ui_g_Ungroup2_MY_Control_state = (ui_interface_round_t*)&(ui_g_Ungroup2_0.data[2]);
ui_interface_number_t *ui_g_Ungroup2_L_track_state = (ui_interface_number_t*)&(ui_g_Ungroup2_0.data[3]);
ui_interface_number_t *ui_g_Ungroup2_R_track_state = (ui_interface_number_t*)&(ui_g_Ungroup2_0.data[4]);
ui_interface_number_t *ui_g_Ungroup2_L_front_state = (ui_interface_number_t*)&(ui_g_Ungroup2_0.data[5]);
ui_interface_number_t *ui_g_Ungroup2_R_front_state = (ui_interface_number_t*)&(ui_g_Ungroup2_0.data[6]);

void _ui_init_g_Ungroup2_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup2_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup2_0.data[i].figure_name[1] = 2;
        ui_g_Ungroup2_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup2_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Ungroup2_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup2_L_behind_state->figure_type = 6;
    ui_g_Ungroup2_L_behind_state->operate_type = 1;
    ui_g_Ungroup2_L_behind_state->layer = 0;
    ui_g_Ungroup2_L_behind_state->color = 0;
    ui_g_Ungroup2_L_behind_state->start_x = 80;
    ui_g_Ungroup2_L_behind_state->start_y = 635;
    ui_g_Ungroup2_L_behind_state->width = 4;
    ui_g_Ungroup2_L_behind_state->font_size = 40;
    ui_g_Ungroup2_L_behind_state->number = 0;

    ui_g_Ungroup2_R_behind_state->figure_type = 6;
    ui_g_Ungroup2_R_behind_state->operate_type = 1;
    ui_g_Ungroup2_R_behind_state->layer = 0;
    ui_g_Ungroup2_R_behind_state->color = 0;
    ui_g_Ungroup2_R_behind_state->start_x = 230;
    ui_g_Ungroup2_R_behind_state->start_y = 635;
    ui_g_Ungroup2_R_behind_state->width = 4;
    ui_g_Ungroup2_R_behind_state->font_size = 40;
    ui_g_Ungroup2_R_behind_state->number = 0;

    ui_g_Ungroup2_MY_Control_state->figure_type = 2;
    ui_g_Ungroup2_MY_Control_state->operate_type = 1;
    ui_g_Ungroup2_MY_Control_state->layer = 0;
    ui_g_Ungroup2_MY_Control_state->color = 0;
    ui_g_Ungroup2_MY_Control_state->start_x = 245;
    ui_g_Ungroup2_MY_Control_state->start_y = 520;
    ui_g_Ungroup2_MY_Control_state->width = 10;
    ui_g_Ungroup2_MY_Control_state->r = 20;

    ui_g_Ungroup2_L_track_state->figure_type = 6;
    ui_g_Ungroup2_L_track_state->operate_type = 1;
    ui_g_Ungroup2_L_track_state->layer = 0;
    ui_g_Ungroup2_L_track_state->color = 0;
    ui_g_Ungroup2_L_track_state->start_x = 75;
    ui_g_Ungroup2_L_track_state->start_y = 855;
    ui_g_Ungroup2_L_track_state->width = 4;
    ui_g_Ungroup2_L_track_state->font_size = 40;
    ui_g_Ungroup2_L_track_state->number = 0;

    ui_g_Ungroup2_R_track_state->figure_type = 6;
    ui_g_Ungroup2_R_track_state->operate_type = 1;
    ui_g_Ungroup2_R_track_state->layer = 0;
    ui_g_Ungroup2_R_track_state->color = 0;
    ui_g_Ungroup2_R_track_state->start_x = 230;
    ui_g_Ungroup2_R_track_state->start_y = 855;
    ui_g_Ungroup2_R_track_state->width = 4;
    ui_g_Ungroup2_R_track_state->font_size = 40;
    ui_g_Ungroup2_R_track_state->number = 0;

    ui_g_Ungroup2_L_front_state->figure_type = 6;
    ui_g_Ungroup2_L_front_state->operate_type = 1;
    ui_g_Ungroup2_L_front_state->layer = 0;
    ui_g_Ungroup2_L_front_state->color = 0;
    ui_g_Ungroup2_L_front_state->start_x = 80;
    ui_g_Ungroup2_L_front_state->start_y = 750;
    ui_g_Ungroup2_L_front_state->width = 4;
    ui_g_Ungroup2_L_front_state->font_size = 40;
    ui_g_Ungroup2_L_front_state->number = 0;

    ui_g_Ungroup2_R_front_state->figure_type = 6;
    ui_g_Ungroup2_R_front_state->operate_type = 1;
    ui_g_Ungroup2_R_front_state->layer = 0;
    ui_g_Ungroup2_R_front_state->color = 0;
    ui_g_Ungroup2_R_front_state->start_x = 230;
    ui_g_Ungroup2_R_front_state->start_y = 750;
    ui_g_Ungroup2_R_front_state->width = 4;
    ui_g_Ungroup2_R_front_state->font_size = 40;
    ui_g_Ungroup2_R_front_state->number = 0;


    ui_proc_7_frame(&ui_g_Ungroup2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup2_0, sizeof(ui_g_Ungroup2_0));
}

void _ui_update_g_Ungroup2_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup2_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Ungroup2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup2_0, sizeof(ui_g_Ungroup2_0));
}

void _ui_remove_g_Ungroup2_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup2_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Ungroup2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup2_0, sizeof(ui_g_Ungroup2_0));
}


void ui_init_g_Ungroup2(void) {
    _ui_init_g_Ungroup2_0();
}

void ui_update_g_Ungroup2(void) {
    _ui_update_g_Ungroup2_0();
}

void ui_remove_g_Ungroup2(void) {
    _ui_remove_g_Ungroup2_0();
}

ui_7_frame_t ui_g_Ungroup3_0;

ui_interface_round_t *ui_g_Ungroup3_Big_Pitch = (ui_interface_round_t*)&(ui_g_Ungroup3_0.data[0]);
ui_interface_rect_t *ui_g_Ungroup3_yaw = (ui_interface_rect_t*)&(ui_g_Ungroup3_0.data[1]);
ui_interface_round_t *ui_g_Ungroup3_smoll_pitch = (ui_interface_round_t*)&(ui_g_Ungroup3_0.data[2]);
ui_interface_round_t *ui_g_Ungroup3_big_roll = (ui_interface_round_t*)&(ui_g_Ungroup3_0.data[3]);
ui_interface_round_t *ui_g_Ungroup3_end_pitch = (ui_interface_round_t*)&(ui_g_Ungroup3_0.data[4]);
ui_interface_round_t *ui_g_Ungroup3_end_roll = (ui_interface_round_t*)&(ui_g_Ungroup3_0.data[5]);
ui_interface_round_t *ui_g_Ungroup3_claw_motor = (ui_interface_round_t*)&(ui_g_Ungroup3_0.data[6]);

void _ui_init_g_Ungroup3_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup3_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup3_0.data[i].figure_name[1] = 3;
        ui_g_Ungroup3_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup3_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Ungroup3_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup3_Big_Pitch->figure_type = 2;
    ui_g_Ungroup3_Big_Pitch->operate_type = 1;
    ui_g_Ungroup3_Big_Pitch->layer = 0;
    ui_g_Ungroup3_Big_Pitch->color = 2;
    ui_g_Ungroup3_Big_Pitch->start_x = 1730;
    ui_g_Ungroup3_Big_Pitch->start_y = 447;
    ui_g_Ungroup3_Big_Pitch->width = 5;
    ui_g_Ungroup3_Big_Pitch->r = 45;

    ui_g_Ungroup3_yaw->figure_type = 1;
    ui_g_Ungroup3_yaw->operate_type = 1;
    ui_g_Ungroup3_yaw->layer = 0;
    ui_g_Ungroup3_yaw->color = 2;
    ui_g_Ungroup3_yaw->start_x = 1672;
    ui_g_Ungroup3_yaw->start_y = 316;
    ui_g_Ungroup3_yaw->width = 5;
    ui_g_Ungroup3_yaw->end_x = 1797;
    ui_g_Ungroup3_yaw->end_y = 388;

    ui_g_Ungroup3_smoll_pitch->figure_type = 2;
    ui_g_Ungroup3_smoll_pitch->operate_type = 1;
    ui_g_Ungroup3_smoll_pitch->layer = 0;
    ui_g_Ungroup3_smoll_pitch->color = 2;
    ui_g_Ungroup3_smoll_pitch->start_x = 1821;
    ui_g_Ungroup3_smoll_pitch->start_y = 584;
    ui_g_Ungroup3_smoll_pitch->width = 5;
    ui_g_Ungroup3_smoll_pitch->r = 45;

    ui_g_Ungroup3_big_roll->figure_type = 2;
    ui_g_Ungroup3_big_roll->operate_type = 1;
    ui_g_Ungroup3_big_roll->layer = 0;
    ui_g_Ungroup3_big_roll->color = 2;
    ui_g_Ungroup3_big_roll->start_x = 1822;
    ui_g_Ungroup3_big_roll->start_y = 753;
    ui_g_Ungroup3_big_roll->width = 5;
    ui_g_Ungroup3_big_roll->r = 45;

    ui_g_Ungroup3_end_pitch->figure_type = 2;
    ui_g_Ungroup3_end_pitch->operate_type = 1;
    ui_g_Ungroup3_end_pitch->layer = 0;
    ui_g_Ungroup3_end_pitch->color = 2;
    ui_g_Ungroup3_end_pitch->start_x = 1669;
    ui_g_Ungroup3_end_pitch->start_y = 822;
    ui_g_Ungroup3_end_pitch->width = 5;
    ui_g_Ungroup3_end_pitch->r = 45;

    ui_g_Ungroup3_end_roll->figure_type = 2;
    ui_g_Ungroup3_end_roll->operate_type = 1;
    ui_g_Ungroup3_end_roll->layer = 0;
    ui_g_Ungroup3_end_roll->color = 2;
    ui_g_Ungroup3_end_roll->start_x = 1562;
    ui_g_Ungroup3_end_roll->start_y = 823;
    ui_g_Ungroup3_end_roll->width = 5;
    ui_g_Ungroup3_end_roll->r = 45;

    ui_g_Ungroup3_claw_motor->figure_type = 2;
    ui_g_Ungroup3_claw_motor->operate_type = 1;
    ui_g_Ungroup3_claw_motor->layer = 0;
    ui_g_Ungroup3_claw_motor->color = 2;
    ui_g_Ungroup3_claw_motor->start_x = 1572;
    ui_g_Ungroup3_claw_motor->start_y = 644;
    ui_g_Ungroup3_claw_motor->width = 5;
    ui_g_Ungroup3_claw_motor->r = 45;


    ui_proc_7_frame(&ui_g_Ungroup3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup3_0, sizeof(ui_g_Ungroup3_0));
}

void _ui_update_g_Ungroup3_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup3_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Ungroup3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup3_0, sizeof(ui_g_Ungroup3_0));
}

void _ui_remove_g_Ungroup3_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup3_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Ungroup3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup3_0, sizeof(ui_g_Ungroup3_0));
}


void ui_init_g_Ungroup3(void) {
    _ui_init_g_Ungroup3_0();
}

void ui_update_g_Ungroup3(void) {
    _ui_update_g_Ungroup3_0();
}

void ui_remove_g_Ungroup3(void) {
    _ui_remove_g_Ungroup3_0();
}

ui_7_frame_t ui_g_Ungroup4_0;

ui_interface_number_t *ui_g_Ungroup4_big_pitch_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[0]);
ui_interface_number_t *ui_g_Ungroup4_yaw_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[1]);
ui_interface_number_t *ui_g_Ungroup4_smoll_pitch_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[2]);
ui_interface_number_t *ui_g_Ungroup4_big_roll_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[3]);
ui_interface_number_t *ui_g_Ungroup4_end_pitch_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[4]);
ui_interface_number_t *ui_g_Ungroup4_end_roll_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[5]);
ui_interface_number_t *ui_g_Ungroup4_claw_state = (ui_interface_number_t*)&(ui_g_Ungroup4_0.data[6]);

void _ui_init_g_Ungroup4_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup4_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup4_0.data[i].figure_name[1] = 4;
        ui_g_Ungroup4_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup4_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Ungroup4_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup4_big_pitch_state->figure_type = 6;
    ui_g_Ungroup4_big_pitch_state->operate_type = 1;
    ui_g_Ungroup4_big_pitch_state->layer = 0;
    ui_g_Ungroup4_big_pitch_state->color = 0;
    ui_g_Ungroup4_big_pitch_state->start_x = 1718;
    ui_g_Ungroup4_big_pitch_state->start_y = 467;
    ui_g_Ungroup4_big_pitch_state->width = 3;
    ui_g_Ungroup4_big_pitch_state->font_size = 30;
    ui_g_Ungroup4_big_pitch_state->number = 0;

    ui_g_Ungroup4_yaw_state->figure_type = 6;
    ui_g_Ungroup4_yaw_state->operate_type = 1;
    ui_g_Ungroup4_yaw_state->layer = 0;
    ui_g_Ungroup4_yaw_state->color = 0;
    ui_g_Ungroup4_yaw_state->start_x = 1719;
    ui_g_Ungroup4_yaw_state->start_y = 372;
    ui_g_Ungroup4_yaw_state->width = 3;
    ui_g_Ungroup4_yaw_state->font_size = 30;
    ui_g_Ungroup4_yaw_state->number = 0;

    ui_g_Ungroup4_smoll_pitch_state->figure_type = 6;
    ui_g_Ungroup4_smoll_pitch_state->operate_type = 1;
    ui_g_Ungroup4_smoll_pitch_state->layer = 0;
    ui_g_Ungroup4_smoll_pitch_state->color = 0;
    ui_g_Ungroup4_smoll_pitch_state->start_x = 1810;
    ui_g_Ungroup4_smoll_pitch_state->start_y = 602;
    ui_g_Ungroup4_smoll_pitch_state->width = 3;
    ui_g_Ungroup4_smoll_pitch_state->font_size = 30;
    ui_g_Ungroup4_smoll_pitch_state->number = 0;

    ui_g_Ungroup4_big_roll_state->figure_type = 6;
    ui_g_Ungroup4_big_roll_state->operate_type = 1;
    ui_g_Ungroup4_big_roll_state->layer = 0;
    ui_g_Ungroup4_big_roll_state->color = 0;
    ui_g_Ungroup4_big_roll_state->start_x = 1810;
    ui_g_Ungroup4_big_roll_state->start_y = 773;
    ui_g_Ungroup4_big_roll_state->width = 3;
    ui_g_Ungroup4_big_roll_state->font_size = 30;
    ui_g_Ungroup4_big_roll_state->number = 0;

    ui_g_Ungroup4_end_pitch_state->figure_type = 6;
    ui_g_Ungroup4_end_pitch_state->operate_type = 1;
    ui_g_Ungroup4_end_pitch_state->layer = 0;
    ui_g_Ungroup4_end_pitch_state->color = 0;
    ui_g_Ungroup4_end_pitch_state->start_x = 1656;
    ui_g_Ungroup4_end_pitch_state->start_y = 840;
    ui_g_Ungroup4_end_pitch_state->width = 3;
    ui_g_Ungroup4_end_pitch_state->font_size = 30;
    ui_g_Ungroup4_end_pitch_state->number = 0;

    ui_g_Ungroup4_end_roll_state->figure_type = 6;
    ui_g_Ungroup4_end_roll_state->operate_type = 1;
    ui_g_Ungroup4_end_roll_state->layer = 0;
    ui_g_Ungroup4_end_roll_state->color = 0;
    ui_g_Ungroup4_end_roll_state->start_x = 1551;
    ui_g_Ungroup4_end_roll_state->start_y = 843;
    ui_g_Ungroup4_end_roll_state->width = 3;
    ui_g_Ungroup4_end_roll_state->font_size = 30;
    ui_g_Ungroup4_end_roll_state->number = 0;

    ui_g_Ungroup4_claw_state->figure_type = 6;
    ui_g_Ungroup4_claw_state->operate_type = 1;
    ui_g_Ungroup4_claw_state->layer = 0;
    ui_g_Ungroup4_claw_state->color = 0;
    ui_g_Ungroup4_claw_state->start_x = 1557;
    ui_g_Ungroup4_claw_state->start_y = 665;
    ui_g_Ungroup4_claw_state->width = 3;
    ui_g_Ungroup4_claw_state->font_size = 30;
    ui_g_Ungroup4_claw_state->number = 0;


    ui_proc_7_frame(&ui_g_Ungroup4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup4_0, sizeof(ui_g_Ungroup4_0));
}

void _ui_update_g_Ungroup4_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup4_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Ungroup4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup4_0, sizeof(ui_g_Ungroup4_0));
}

void _ui_remove_g_Ungroup4_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup4_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Ungroup4_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup4_0, sizeof(ui_g_Ungroup4_0));
}


void ui_init_g_Ungroup4(void){
    _ui_init_g_Ungroup4_0();
}

void ui_update_g_Ungroup4(void) {
    _ui_update_g_Ungroup4_0();
}

void ui_remove_g_Ungroup4(void) {
    _ui_remove_g_Ungroup4_0();
}

ui_7_frame_t ui_g_Ungroup5_0;

ui_interface_line_t *ui_g_Ungroup5_Big = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[0]);
ui_interface_line_t *ui_g_Ungroup5_smoll = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[1]);
ui_interface_line_t *ui_g_Ungroup5_end = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[2]);
ui_interface_line_t *ui_g_Ungroup5_claw = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[3]);
ui_interface_line_t *ui_g_Ungroup5_L = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[4]);
ui_interface_line_t *ui_g_Ungroup5_R = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[5]);
ui_interface_line_t *ui_g_Ungroup5_car = (ui_interface_line_t*)&(ui_g_Ungroup5_0.data[6]);

void _ui_init_g_Ungroup5_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup5_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup5_0.data[i].figure_name[1] = 5;
        ui_g_Ungroup5_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup5_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Ungroup5_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup5_Big->figure_type = 0;
    ui_g_Ungroup5_Big->operate_type = 1;
    ui_g_Ungroup5_Big->layer = 0;
    ui_g_Ungroup5_Big->color = 2;
    ui_g_Ungroup5_Big->start_x = 1775;
    ui_g_Ungroup5_Big->start_y = 476;
    ui_g_Ungroup5_Big->width = 5;
    ui_g_Ungroup5_Big->end_x = 1843;
    ui_g_Ungroup5_Big->end_y = 531;

    ui_g_Ungroup5_smoll->figure_type = 0;
    ui_g_Ungroup5_smoll->operate_type = 1;
    ui_g_Ungroup5_smoll->layer = 0;
    ui_g_Ungroup5_smoll->color = 2;
    ui_g_Ungroup5_smoll->start_x = 1822;
    ui_g_Ungroup5_smoll->start_y = 645;
    ui_g_Ungroup5_smoll->width = 5;
    ui_g_Ungroup5_smoll->end_x = 1822;
    ui_g_Ungroup5_smoll->end_y = 699;

    ui_g_Ungroup5_end->figure_type = 0;
    ui_g_Ungroup5_end->operate_type = 1;
    ui_g_Ungroup5_end->layer = 0;
    ui_g_Ungroup5_end->color = 2;
    ui_g_Ungroup5_end->start_x = 1723;
    ui_g_Ungroup5_end->start_y = 831;
    ui_g_Ungroup5_end->width = 5;
    ui_g_Ungroup5_end->end_x = 1784;
    ui_g_Ungroup5_end->end_y = 803;

    ui_g_Ungroup5_claw->figure_type = 0;
    ui_g_Ungroup5_claw->operate_type = 1;
    ui_g_Ungroup5_claw->layer = 0;
    ui_g_Ungroup5_claw->color = 2;
    ui_g_Ungroup5_claw->start_x = 1571;
    ui_g_Ungroup5_claw->start_y = 707;
    ui_g_Ungroup5_claw->width = 5;
    ui_g_Ungroup5_claw->end_x = 1569;
    ui_g_Ungroup5_claw->end_y = 765;

    ui_g_Ungroup5_L->figure_type = 0;
    ui_g_Ungroup5_L->operate_type = 1;
    ui_g_Ungroup5_L->layer = 0;
    ui_g_Ungroup5_L->color = 0;
    ui_g_Ungroup5_L->start_x = 1142;
    ui_g_Ungroup5_L->start_y = 540;
    ui_g_Ungroup5_L->width = 8;
    ui_g_Ungroup5_L->end_x = 1279;
    ui_g_Ungroup5_L->end_y = 12;

    ui_g_Ungroup5_R->figure_type = 0;
    ui_g_Ungroup5_R->operate_type = 1;
    ui_g_Ungroup5_R->layer = 0;
    ui_g_Ungroup5_R->color = 0;
    ui_g_Ungroup5_R->start_x = 650;
    ui_g_Ungroup5_R->start_y = 572;
    ui_g_Ungroup5_R->width = 8;
    ui_g_Ungroup5_R->end_x = 441;
    ui_g_Ungroup5_R->end_y = 4;

    ui_g_Ungroup5_car->figure_type = 0;
    ui_g_Ungroup5_car->operate_type = 1;
    ui_g_Ungroup5_car->layer = 0;
    ui_g_Ungroup5_car->color = 2;
    ui_g_Ungroup5_car->start_x = 153;
    ui_g_Ungroup5_car->start_y = 672;
    ui_g_Ungroup5_car->width = 5;
    ui_g_Ungroup5_car->end_x = 194;
    ui_g_Ungroup5_car->end_y = 674;


    ui_proc_7_frame(&ui_g_Ungroup5_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup5_0, sizeof(ui_g_Ungroup5_0));
}

void _ui_update_g_Ungroup5_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup5_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Ungroup5_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup5_0, sizeof(ui_g_Ungroup5_0));
}

void _ui_remove_g_Ungroup5_0(void) {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup5_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Ungroup5_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup5_0, sizeof(ui_g_Ungroup5_0));
}


void ui_init_g_Ungroup5(void) {
    _ui_init_g_Ungroup5_0();
}

void ui_update_g_Ungroup5(void) {
    _ui_update_g_Ungroup5_0();
}

void ui_remove_g_Ungroup5(void) {
    _ui_remove_g_Ungroup5_0();
}

