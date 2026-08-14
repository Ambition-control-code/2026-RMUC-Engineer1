#include "Gravity.h"
#include "math.h"
#include "arm_math.h"


typedef struct { float x, y, z; } Vec3;
typedef struct { float m[3][3]; } Mat3;

// ¦V¶q¥[ªk/ú£ªk/¤e­¼ (?™m¨ç?¡AFPUª½±µ?¦æ)
static inline Vec3 Vec3_Add(Vec3 a, Vec3 b) { return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 Vec3_Sub(Vec3 a, Vec3 b) { return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 Vec3_Cross(Vec3 a, Vec3 b)
{
    return (Vec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

// ¦V¶q˜ò­¼ (¨Ï¥Î arm_math.h ªº DSP É¬¤Æ¨ç?)
static inline float Vec3_Dot(Vec3 a, Vec3 b) {
    float result;
    float pSrcA[3] = {a.x, a.y, a.z};
    float pSrcB[3] = {b.x, b.y, b.z};
    arm_dot_prod_f32(pSrcA, pSrcB, 3, &result);
    return result;
}

// 3x3¯x?­¼ 3x1¦V¶q
static inline Vec3 Mat3_MultVec(Mat3 M, Vec3 v) {
    return (Vec3){
        M.m[0][0]*v.x + M.m[0][1]*v.y + M.m[0][2]*v.z,
        M.m[1][0]*v.x + M.m[1][1]*v.y + M.m[1][2]*v.z,
        M.m[2][0]*v.x + M.m[2][1]*v.y + M.m[2][2]*v.z
    };
}

// 3x3¯x?¬Û­¼
static inline Mat3 Mat3_Mult(Mat3 A, Mat3 B) {
    Mat3 C = {0};
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            for(int k=0; k<3; k++)
                C.m[i][j] += A.m[i][k] * B.m[k][j];
    return C;
}

// ¥Í¦¨±Û?¯x? (¨Ï¥Î arm_math.h ªºÌå³t¤T¨¤¨ç?)
static inline Mat3 Mat3_RotX(float angle) {
    float c = arm_cos_f32(angle), s = arm_sin_f32(angle);
    return (Mat3){{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}
static inline Mat3 Mat3_RotY(float angle) {
    float c = arm_cos_f32(angle), s = arm_sin_f32(angle);
    return (Mat3){{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}
static inline Mat3 Mat3_RotZ(float angle) {
    float c = arm_cos_f32(angle), s = arm_sin_f32(angle);
    return (Mat3){{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

// ==========================================
// 2. Éó±ñÁuª«²z‰ë?°t¸m
// ==========================================
#define G_ACC 9.81f

// ¦U?§ıªº?¶q (?¦ì: kg) - »İ¦Û¦æ­×§ï
#define MASS_L1 1.5f   // ¤jÁu
#define MASS_L2 1.0f   // ¤pÁu (§t¤jRoll‰mÉó)
#define MASS_L3 0.6f   // ¥½ºİ?¤ö?¥ó

// ?¤ß¬Û?¤_«e??ªº¥»¦a§¤? (¹s¦ì°²‰z¤âÁuªuX?¤ô¥­¦ùª½)
const Vec3 CoM_L1 = {0.15f, 0.0f, 0.0f}; // ¤jÁu?¤ß¶Z¤jPitch?0.15m
const Vec3 CoM_L2 = {0.15f, 0.0f, 0.0f}; // ¤pÁu?¤ß¶Z¤pPitch?0.15m
const Vec3 CoM_L3 = {0.05f, 0.0f, 0.0f}; // ¥½ºİ?¤ß¶Z¥½ºİPitch?0.05m

// ?§ı‹ë«×¦V¶q (??¤§šVªº°¾²¾)
const Vec3 Link_1 = {0.30f, 0.0f, 0.0f}; // ¤jPitch -> ¤pPitch
const Vec3 Link_2 = {0.30f, 0.0f, 0.0f}; // ¤pPitch -> ¥½ºİPitch (¥]§t¤F¤jRoll¬q)
const Vec3 Link_3 = {0.10f, 0.0f, 0.0f}; // ¥½ºİPitch -> ?¤ö¥½ºİ

// ‰mÉó§á¯x±`? (Nm/A) - ¥Î¤_§â¤O¯x??‰m¬y
#define KT_DM4310_BIG_PITCH   0.25f  // }§®4310
#define KT_DM4310_SMALL_PITCH 0.25f  // }§®4310
#define KT_M3508_ROLL         0.18f  // 3508
#define KT_M6020_END_PITCH    0.30f  // 6020
#define KT_M3508_END_ROLL     0.18f  // 3508

#define DAMP_BIG_PITCH   0.20f  // ¤j??¤OÁu‹ë¡Aªı¥§?¤j˜ò
#define DAMP_SMALL_PITCH 0.15f 
#define DAMP_ROLL        0.05f  // ¥u¦³?‰V?¶q¡Aªı¥§?¤p˜ò
#define DAMP_END_PITCH   0.05f

// ™]¥X¡G¤»ŸÄ‰mÉóªºœ|?‰m¬y´Á±æ­È (?¦ì: A)
float gravity_comp_current[6] = {0};

// ==========================================
// 3. ®Ö¤ß?ºâ¡G`‰V‰aÉO‰V¤O‰a­«¤O¯x¸Ñºâ
// ==========================================
void Calculate_Gravity_Compensation(void)
{
    // [0]=Yaw, [1]=¤jPitch, [2]=¤pPitch, [3]=¤jRoll, [4]=¥½ºİPitch, [5]=¥½ºİRoll
    static float joint_angles[6] = {0}; // ŸÏ??¾¹?¨úªº?«e??¨¤«× (?¦ì: ©·«×)
//    static float joint_velocities[6] = {0}; // ŸÏ??¾¹?¨úªº?«e??³t«× (?¦ì: ©·«×/¬í)




    

    // ¥@¬É§¤?¨t­ì˜òÉO«º?
    Mat3 R0 = {{{1,0,0}, {0,1,0}, {0,0,1}}};
    Vec3 P0 = {0,0,0}; 

    // --- ¥¿¦V`‰V‰a¡G?ºâ¦U??ªº¥@¬É§¤?ÉO±Û?? ---
    
    // 1. Yaw (?Z)
    Mat3 R1 = Mat3_Mult(R0, Mat3_RotZ(joint_angles[0])); 
    Vec3 P1 = P0;
    
    // 2. ¤jPitch (?Y)
    Mat3 R2 = Mat3_Mult(R1, Mat3_RotY(joint_angles[1]));
    Vec3 P2 = P1;
    Vec3 Z_axis_2 = Mat3_MultVec(R2, (Vec3){0,1,0}); // ?¨úY?¦b¥@¬É§¤?¨tªº¤è¦V
    Vec3 P_com_L1 = Vec3_Add(P2, Mat3_MultVec(R2, CoM_L1)); // ¤jÁu?¤ß¥@¬É§¤?

    // 3. ¤pPitch (?Y)
    Mat3 R3 = Mat3_Mult(R2, Mat3_RotY(joint_angles[2]));
    Vec3 P3 = Vec3_Add(P2, Mat3_MultVec(R2, Link_1)); 
    Vec3 Z_axis_3 = Mat3_MultVec(R3, (Vec3){0,1,0});
    Vec3 P_com_L2 = Vec3_Add(P3, Mat3_MultVec(R3, CoM_L2)); // ¤pÁu?¤ß

    // 4. ¤jRoll (?X)
    Mat3 R4 = Mat3_Mult(R3, Mat3_RotX(joint_angles[3]));
    Vec3 P4 = P3;
    Vec3 Z_axis_4 = Mat3_MultVec(R4, (Vec3){1,0,0}); // Roll¬O?X?

    // 5. ¥½ºİPitch (?Y)
    Mat3 R5 = Mat3_Mult(R4, Mat3_RotY(joint_angles[4]));
    Vec3 P5 = Vec3_Add(P4, Mat3_MultVec(R4, Link_2));
    Vec3 Z_axis_5 = Mat3_MultVec(R5, (Vec3){0,1,0});
    Vec3 P_com_L3 = Vec3_Add(P5, Mat3_MultVec(R5, CoM_L3)); // ¥½ºİ?¤ß

    // --- °f¦V‰V¤O‰a¡G?ºâ­«¤O¯x (Torque = R x F) ---
    float torque[6] = {0};
//    float damp_torque[6] = {0}; // ªı¥§¤O¯x (¦pªG»İ­nªº?¡A¥i¥H®ÚÕu??³t«×?ºâ)
    // ­«¤O¥Ú¶q (ùÚ©w´Â¤U)
    Vec3 F_L1 = {0, 0, MASS_L1 * -G_ACC};
    Vec3 F_L2 = {0, 0, MASS_L2 * -G_ACC};
    Vec3 F_L3 = {0, 0, MASS_L3 * -G_ACC};

    // ?ºâ J2 (¤jPitch) ©Ó¨üªº­«¤O¯x (¤ä‰ML1, L2, L3)
    Vec3 T2_L1 = Vec3_Cross(Vec3_Sub(P_com_L1, P2), F_L1);
    Vec3 T2_L2 = Vec3_Cross(Vec3_Sub(P_com_L2, P2), F_L2);
    Vec3 T2_L3 = Vec3_Cross(Vec3_Sub(P_com_L3, P2), F_L3);
    Vec3 T2_total = Vec3_Add(Vec3_Add(T2_L1, T2_L2), T2_L3);
    torque[1] = Vec3_Dot(T2_total, Z_axis_2);

    // ?ºâ J3 (¤pPitch) ©Ó¨üªº­«¤O¯x (¤ä‰ML2, L3)
    Vec3 T3_L2 = Vec3_Cross(Vec3_Sub(P_com_L2, P3), F_L2);
    Vec3 T3_L3 = Vec3_Cross(Vec3_Sub(P_com_L3, P3), F_L3);
    Vec3 T3_total = Vec3_Add(T3_L2, T3_L3);
    torque[2] = Vec3_Dot(T3_total, Z_axis_3);

    // ?ºâ J4 (¤jRoll) ©Ó¨üªº­«¤O¯x (¤ä‰ML3¡A§Y¥½ºİ?¦±??Roll?¥Íªº§á¤O)
    Vec3 T4_L3 = Vec3_Cross(Vec3_Sub(P_com_L3, P4), F_L3);
    torque[3] = Vec3_Dot(T4_L3, Z_axis_4);

    // ?ºâ J5 (¥½ºİPitch) ©Ó¨üªº­«¤O¯x (¤ä‰ML3)
    Vec3 T5_L3 = Vec3_Cross(Vec3_Sub(P_com_L3, P5), F_L3);
    torque[4] = Vec3_Dot(T5_L3, Z_axis_5);

    // Yaw(0) ©M ¥½ºİRoll(5) ¤@¯ë¤£¨ü©ú???­«¤O¯x
    torque[0] = 0.0f;
    torque[5] = 0.0f;

    // --- ?¤O¯x (Nm) ???‰mÉóœ|?‰m¬y (A) ---
    // ²Å­»İ­n®ÚÕu§A‰mÉóªº¦wÆ¤è¦V©M??¾¹©wúå?¦æ?? (¥¿?­??)
    gravity_comp_current[0] = 0.0f; 
    gravity_comp_current[1] = -torque[1] / KT_DM4310_BIG_PITCH;
    gravity_comp_current[2] = -torque[2] / KT_DM4310_SMALL_PITCH;
    gravity_comp_current[3] = -torque[3] / KT_M3508_ROLL;
    gravity_comp_current[4] = -torque[4] / KT_M6020_END_PITCH;
    gravity_comp_current[5] = 0.0f;
}







