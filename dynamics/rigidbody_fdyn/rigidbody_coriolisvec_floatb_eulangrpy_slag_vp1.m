% Calculate vector of centrifugal and coriolis load on the joints for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [0x1]
%   Joint Angles [rad]
% qD [0x1]
%   Joint Velocities [rad/s]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of 
%   r_base (3x1 Base position in world frame) and 
%   phi_base (3x1)
% a_mdh, d_mdh, q_offset_mdh [0x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [1x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% tauc [(6+0)x1]
%   base wrench and joint torques required to compensate coriolis and centrifugal load

function tauc = rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1(q, qD, phi_base, xD_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh, Icges_num_mdh)
%%Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: q has to be [0x1] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: qD has to be [0x1] double');
assert(isa(phi_base,'double') && isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: phi_base has to be [3x1] double');
assert(isa(xD_base,'double') && isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: xD_base has to be [6x1] double');
assert(isa(alpha_mdh,'double') && isreal(alpha_mdh) && all(size(alpha_mdh) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: alpha_mdh has to be [0x1] double');
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: a_mdh has to be [0x1] double');
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: d_mdh has to be [0x1] double');
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: q_offset_mdh has to be [0x1] double');
assert(isa(b_mdh,'double') && isreal(b_mdh) && all(size(b_mdh) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: b_mdh has to be [0x1] double');
assert(isa(beta_mdh,'double') && isreal(beta_mdh) && all(size(beta_mdh) == [0 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: beta_mdh has to be [0x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [1 1]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: m_num has to be [1x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [1,3]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: rSges_num_mdh has to be [1x3] double');
assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [1 6]), ...
  'rigidbody_coriolisvec_floatb_eulangrpy_slag_vp1: Icges_num_mdh has to be [1x6] double'); 

%% Variable Initialization


alphaxs_base = phi_base(1);
betays_base = phi_base(2);
gammazs_base = phi_base(3);

vxs_base = xD_base(1);
vys_base = xD_base(2);
vzs_base = xD_base(3);
alphaDx_base = xD_base(4);
betaDy_base = xD_base(5);
gammaDz_base = xD_base(6);


M1 = m_num(1);

SX1 = rSges_num_mdh(1,1);
SY1 = rSges_num_mdh(1,2);
SZ1 = rSges_num_mdh(1,3);

XXC1 = Icges_num_mdh(1,1);
XYC1 = Icges_num_mdh(1,4);
XZC1 = Icges_num_mdh(1,5);
YYC1 = Icges_num_mdh(1,2);
YZC1 = Icges_num_mdh(1,6);
ZZC1 = Icges_num_mdh(1,3);

%%Symbolic Calculation
%From rigidbody_coriolisvec_floatb_eulangrpy_par1_matlab.m
t1032 = sin(alphaxs_base);
t1034 = cos(alphaxs_base);
t1036 = cos(betays_base);
t1031 = sin(gammazs_base);
t1035 = sin(betays_base);
t1108 = t1034 * t1035;
t1033 = cos(gammazs_base);
t1112 = t1032 * t1033;
t1021 = t1031 * t1108 + t1112;
t1022 = -t1032 * t1031 + t1033 * t1108;
t1018 = XYC1 * t1022;
t1107 = t1034 * t1036;
t979 = YYC1 * t1021 + YZC1 * t1107 - t1018;
t1017 = XYC1 * t1021;
t981 = XXC1 * t1022 - XZC1 * t1107 - t1017;
t1069 = -t1021 * t979 - t1022 * t981;
t1109 = t1033 * t1034;
t1111 = t1032 * t1035;
t1019 = -t1031 * t1111 + t1109;
t1020 = t1031 * t1034 + t1033 * t1111;
t1110 = t1032 * t1036;
t1125 = XYC1 * t1020;
t977 = YYC1 * t1019 - YZC1 * t1110 + t1125;
t1016 = XYC1 * t1019;
t980 = XXC1 * t1020 - XZC1 * t1110 + t1016;
t1130 = t1019 * t977 + t1020 * t980;
t974 = XZC1 * t1020 + YZC1 * t1019 - ZZC1 * t1110;
t976 = -XZC1 * t1022 + YZC1 * t1021 + ZZC1 * t1107;
t1158 = t1069 + t1130 + (-t1032 * t974 - t1034 * t976) * t1036;
t1096 = t1036 * gammaDz_base;
t1103 = t1034 * betaDy_base;
t1028 = -t1032 * t1096 + t1103;
t1098 = t1035 * gammaDz_base;
t1029 = alphaDx_base + t1098;
t943 = t1021 * t977 - t1022 * t980 + t974 * t1107;
t1071 = XZC1 * t1033 - YZC1 * t1031;
t1009 = ZZC1 * t1035 + t1036 * t1071;
t1123 = XYC1 * t1033;
t1073 = -YYC1 * t1031 + t1123;
t1011 = YZC1 * t1035 + t1036 * t1073;
t1124 = XYC1 * t1031;
t1075 = XXC1 * t1033 - t1124;
t1013 = XZC1 * t1035 + t1036 * t1075;
t958 = t1009 * t1107 + t1021 * t1011 - t1022 * t1013;
t1157 = -t1028 * t943 - t1029 * t958;
t1079 = SX1 * t1033 - SY1 * t1031;
t1015 = t1035 * SZ1 + t1036 * t1079;
t1104 = t1032 * betaDy_base;
t1027 = t1034 * t1096 + t1104;
t988 = -t1022 * SX1 + t1021 * SY1 + SZ1 * t1107;
t1155 = -t1015 * t1027 + t1029 * t988;
t1067 = t1031 * t979 + t1033 * t981;
t946 = t1035 * t976 - t1036 * t1067;
t942 = t1019 * t979 - t1020 * t981 - t1110 * t976;
t1060 = -alphaDx_base + t1098;
t1076 = alphaDx_base * t1096;
t1005 = -t1032 * t1076 - t1060 * t1103;
t1140 = t1005 / 0.2e1;
t944 = t1107 * t976 - t1069;
t1151 = t944 * t1140;
t1004 = -t1034 * t1076 + t1060 * t1104;
t1141 = t1004 / 0.2e1;
t1138 = t1027 / 0.2e1;
t1136 = t1028 / 0.2e1;
t1135 = -t1029 / 0.2e1;
t1134 = t1029 / 0.2e1;
t1077 = betaDy_base * t1096;
t1058 = t1077 / 0.2e1;
t1097 = t1036 * alphaDx_base;
t1090 = t1032 * t1097;
t1102 = t1035 * betaDy_base;
t1091 = t1034 * t1102;
t1043 = -t1090 - t1091;
t1046 = t1034 * t1029;
t1059 = t1035 * alphaDx_base + gammaDz_base;
t1101 = t1036 * betaDy_base;
t1144 = t1032 * t1059 - t1034 * t1101;
t969 = -t1031 * t1144 + t1033 * t1046;
t970 = t1031 * t1046 + t1033 * t1144;
t953 = t970 * SX1 + t969 * SY1 + SZ1 * t1043;
t986 = t1020 * SX1 + t1019 * SY1 - SZ1 * t1110;
t1148 = -t986 * alphaDx_base + t953;
t1089 = t1034 * t1097;
t1093 = t1032 * t1102;
t1042 = -t1089 + t1093;
t971 = -t1029 * t1112 + (-t1032 * t1101 - t1034 * t1059) * t1031;
t972 = t1059 * t1109 + (-t1029 * t1031 + t1033 * t1101) * t1032;
t954 = t972 * SX1 + t971 * SY1 + SZ1 * t1042;
t1147 = -t988 * alphaDx_base - t954;
t957 = -t1009 * t1110 + t1019 * t1011 + t1020 * t1013;
t1146 = t1027 * t942 + t957 * t1029;
t1049 = t1071 * t1035;
t1061 = t1011 * t1031 - t1013 * t1033;
t1068 = t1031 * t977 - t1033 * t980;
t1037 = t1027 * (-t1009 * t1034 + t1067) + t1028 * (t1009 * t1032 + t1068) + t1029 * (ZZC1 * t1036 - t1049 + t1061);
t1145 = t1037 * t1036;
t1072 = -YYC1 * t1033 - t1124;
t1039 = t1027 * (YYC1 * t1022 + t1017 - t981) + t1028 * (-YYC1 * t1020 + t1016 + t980) + t1029 * (t1072 * t1036 + t1013);
t1074 = -XXC1 * t1031 - t1123;
t1143 = t1027 * (-XXC1 * t1021 - t1018 + t979) + t1028 * (-XXC1 * t1019 + t1125 + t977) + t1029 * (-t1074 * t1036 + t1011);
t948 = XZC1 * t972 + YZC1 * t971 + ZZC1 * t1042;
t1047 = -t1036 * t948 + t1102 * t974;
t950 = XYC1 * t972 + YYC1 * t971 + YZC1 * t1042;
t952 = XXC1 * t972 + XYC1 * t971 + XZC1 * t1042;
t925 = t1021 * t950 - t1022 * t952 - t1034 * t1047 - t1090 * t974 + t969 * t977 + t970 * t980;
t947 = XZC1 * t970 + YZC1 * t969 + ZZC1 * t1043;
t1048 = -t1036 * t947 + t1102 * t976;
t949 = XYC1 * t970 + YYC1 * t969 + YZC1 * t1043;
t951 = XXC1 * t970 + XYC1 * t969 + XZC1 * t1043;
t926 = t1021 * t949 - t1022 * t951 - t1034 * t1048 - t1090 * t976 + t969 * t979 - t970 * t981;
t1070 = -XZC1 * t1031 - YZC1 * t1033;
t983 = -betaDy_base * t1049 + (ZZC1 * betaDy_base + t1070 * gammaDz_base) * t1036;
t1050 = t1073 * t1035;
t984 = -betaDy_base * t1050 + (YZC1 * betaDy_base + t1072 * gammaDz_base) * t1036;
t1051 = t1075 * t1035;
t985 = -betaDy_base * t1051 + (XZC1 * betaDy_base + t1074 * gammaDz_base) * t1036;
t935 = t1009 * t1043 + t969 * t1011 + t970 * t1013 + t1021 * t984 - t1022 * t985 + t1107 * t983;
t1142 = t958 * t1058 + t935 * t1134 + t925 * t1136 + t926 * t1138 + t1141 * t943 + t1151;
t1139 = -t1027 / 0.2e1;
t1137 = -t1028 / 0.2e1;
t940 = (t1061 * betaDy_base + t983) * t1035 + (t1009 * betaDy_base + (-t1011 * gammaDz_base + t985) * t1033 + (-t1013 * gammaDz_base - t984) * t1031) * t1036;
t962 = t1035 * t1009 - t1036 * t1061;
t1131 = t940 * t1029 + t962 * t1077;
t929 = (t1068 * betaDy_base + t948) * t1035 + (t974 * betaDy_base + (-t977 * gammaDz_base + t952) * t1033 + (-t980 * gammaDz_base - t950) * t1031) * t1036;
t1116 = t929 * t1028;
t930 = (t1067 * betaDy_base + t947) * t1035 + (t976 * betaDy_base + (-t979 * gammaDz_base + t951) * t1033 + (t981 * gammaDz_base - t949) * t1031) * t1036;
t1115 = t930 * t1027;
t945 = t1035 * t974 - t1036 * t1068;
t1114 = t945 * t1004;
t1113 = t946 * t1005;
t1087 = t944 * alphaDx_base + t925;
t1086 = -t943 * alphaDx_base + t926;
t927 = t1019 * t950 + t1020 * t952 + t1032 * t1047 - t1089 * t974 + t971 * t977 + t972 * t980;
t1085 = t942 * alphaDx_base + t927;
t928 = t1019 * t949 + t1020 * t951 + t1032 * t1048 - t1089 * t976 + t971 * t979 - t972 * t981;
t941 = -t1110 * t974 + t1130;
t1084 = -t941 * alphaDx_base + t928;
t1083 = t946 * alphaDx_base + t929;
t1082 = -t945 * alphaDx_base + t930;
t1053 = t1079 * t1035;
t1078 = -SX1 * t1031 - SY1 * t1033;
t989 = -betaDy_base * t1053 + (SZ1 * betaDy_base + t1078 * gammaDz_base) * t1036;
t938 = -t1004 * t1015 - t1028 * t989 + t1029 * t954 + t1077 * t986;
t960 = vys_base - t1155;
t1081 = t960 * alphaDx_base - t938;
t939 = t1005 * t1015 + t1027 * t989 - t1029 * t953 - t1077 * t988;
t1062 = t1015 * t1028 - t1029 * t986;
t959 = -t1062 + vzs_base;
t1080 = t959 * alphaDx_base + t939;
t1066 = t1032 * t941 - t1034 * t942;
t1065 = t1032 * t943 - t1034 * t944;
t1064 = t1032 * t945 - t1034 * t946;
t1063 = t1032 * t988 + t1034 * t986;
t1045 = t1009 * t1029 + t1027 * t976 + t1028 * t974;
t1044 = t1070 * t1036 * t1029 + (XZC1 * t1021 + YZC1 * t1022) * t1027 + (XZC1 * t1019 - YZC1 * t1020) * t1028;
t1041 = t938 * t986 - t939 * t988 - t960 * t953 + t959 * t954;
t955 = -t1027 * t986 + t1028 * t988 + vxs_base;
t1038 = t955 * t1063 + (-t959 * t1032 - t960 * t1034) * t1015;
t1026 = t1078 * t1036;
t1014 = SZ1 * t1036 - t1053;
t1012 = XZC1 * t1036 - t1051;
t1010 = YZC1 * t1036 - t1050;
t1007 = t1015 * t1034;
t1006 = t1015 * t1032;
t1003 = t1013 * t1034;
t1002 = t1013 * t1032;
t1001 = t1011 * t1034;
t1000 = t1011 * t1032;
t997 = SX1 * t1021 + SY1 * t1022;
t996 = SX1 * t1019 - SY1 * t1020;
t936 = t1009 * t1042 + t971 * t1011 + t972 * t1013 + t1019 * t984 + t1020 * t985 - t1110 * t983;
t934 = t1027 * t946 + t1028 * t945 + t1029 * t962;
t933 = t1004 * t988 - t1005 * t986 - t1027 * t954 + t1028 * t953;
t932 = t1027 * t944 - t1157;
t931 = t1028 * t941 + t1146;
t924 = t941 * t1004 + t942 * t1005 + t928 * t1027 + t927 * t1028 + t936 * t1029 + t1077 * t957;
t1 = [M1 * t933; M1 * t939; M1 * t938; t1116 / 0.2e1 + t1114 / 0.2e1 + t1115 / 0.2e1 + t1113 / 0.2e1 + t957 * t1141 + t936 * t1136 + t958 * t1140 + ((t944 + t1158) * t1028 + t1146) * t1139 + t1131 + (t935 + t931) * t1138 + ((-t941 + t1158) * t1027 + t932 + t1157) * t1137 + (-t960 * t1062 + t1155 * t959 + t1041) * M1; -t934 * t1096 / 0.2e1 + (((-t1010 * t1031 + t1012 * t1033 + t1009) * t1029 + t962 * gammaDz_base + (-t1000 * t1031 + t1002 * t1033 + t974) * t1028 + (t1001 * t1031 - t1003 * t1033 + t976) * t1027) * t1036 + (t1064 * gammaDz_base + t1037) * t1035) * t1135 + ((t1019 * t1010 + t1020 * t1012) * t1029 + (t1019 * t1000 + t1020 * t1002) * t1028 + (-t1019 * t1001 - t1020 * t1003) * t1027 + (t1036 * t957 - t1108 * t942) * gammaDz_base) * t1137 + ((t1021 * t1010 - t1022 * t1012) * t1029 + (t1021 * t1000 - t1022 * t1002) * t1028 + (-t1021 * t1001 + t1022 * t1003) * t1027 + (t1036 * t958 + t1111 * t943) * gammaDz_base) * t1139 + (-t955 * (-t1006 * t1027 - t1007 * t1028) - t960 * (t1029 * t1007 + t1027 * t1014) - t959 * (t1029 * t1006 - t1028 * t1014) - ((t959 * t986 - t960 * t988) * t1036 + t1038 * t1035) * gammaDz_base) * M1 + (t1029 * t932 + t924) * t1034 / 0.2e1 + (t945 * t1058 + t1083 * t1134 + t941 * t1141 + t1085 * t1136 + t943 * t1140 + t1087 * t1138 + ((-t944 * gammaDz_base - t1045) * t1035 + t1145) * t1139 + (t1081 * t1015 + t1148 * t955 + t933 * t988 - t959 * t989) * M1) * t1034 + (t946 * t1058 + t1082 * t1134 + t942 * t1141 + t1084 * t1136 + t1142 + t1151 + t1086 * t1138 + ((t941 * gammaDz_base + t1045) * t1035 - t1145) * t1137 + t931 * t1135 + (t1080 * t1015 + t1147 * t955 - t933 * t986 + t960 * t989) * M1) * t1032; t934 * t1101 / 0.2e1 + t1035 * (t1113 + t1114 + t1115 + t1116 + t1131) / 0.2e1 + (t962 * t1035 - t1036 * t1064) * t1058 + ((t1064 * betaDy_base + t940) * t1035 + (-t1032 * t1083 + t1034 * t1082 + t962 * betaDy_base) * t1036) * t1134 + t931 * t1093 / 0.2e1 - t924 * t1110 / 0.2e1 + (t957 * t1035 - t1036 * t1066) * t1141 + ((t1066 * betaDy_base + t936) * t1035 + (-t1032 * t1085 + t1034 * t1084 + t957 * betaDy_base) * t1036) * t1136 - t932 * t1091 / 0.2e1 + t1107 * t1142 + (t958 * t1035 - t1065 * t1036) * t1140 + ((t1065 * betaDy_base + t935) * t1035 + (-t1032 * t1087 + t1034 * t1086 + t958 * betaDy_base) * t1036) * t1138 + (t1044 * t1035 + (-t1031 * t1039 - t1033 * t1143) * t1036) * t1135 + (t1039 * t1019 - t1020 * t1143 - t1044 * t1110) * t1137 + (t1039 * t1021 + t1022 * t1143 + t1044 * t1107) * t1139 - (t1032 * t932 + t1034 * t931) * t1097 / 0.2e1 + ((t1038 * betaDy_base + t1041) * t1035 + (-t933 * t1063 + t955 * (-t1032 * t1148 + t1034 * t1147) + t960 * (t1034 * t989 - t988 * betaDy_base) + t959 * (t1032 * t989 + t986 * betaDy_base) + (-t1032 * t1081 + t1034 * t1080) * t1015) * t1036 - t955 * (-t1027 * t996 + t1028 * t997) - t960 * (t1026 * t1027 - t1029 * t997) - t959 * (-t1026 * t1028 + t1029 * t996)) * M1;];
tauc  = t1 (:);
