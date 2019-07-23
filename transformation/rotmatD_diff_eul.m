% Ableitung der Rotationsmatrix nach den Euler-Winkeln und der Zeit
%
% Eingabe:
% phi [3x1]:
%   Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der Euler-Winkel
% conv [1x1] (uint8)
%   Nummer der Euler-Winkel-Konvention
%   Siehe euler_angle_properties.m
%
% Ausgabe:
% GradMatD [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix
%   nach den sie erzeugenden Euler-Winkeln und dieses nochmal nach der Zeit

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function GradMatD = rotmatD_diff_eul(phi, phiD, conv)
%% Init
%#codegen
%$cgargs {zeros(3,1), zeros(3,1), uint8(2)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmatD_diff_eul: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'rotmatD_diff_eul: phiD has to be [3x1] (double)');
assert(isa(conv,'uint8') && isscalar(conv), 'rotmatD_diff_eul: Number of Euler convention has to be [1x1] uint8');
phi1_s=phi(1);phi2_s=phi(2);phi3_s=phi(3);
phi1D_s=phiD(1);phi2D_s=phiD(2);phi3D_s=phiD(3);
%% Berechnung
switch conv
case 1 % xyx
% aus codeexport/rotmatD_diff_eulxyx_matlab.m (euler_angle_calculations.mw)
t639 = cos(phi3_s);
t641 = cos(phi1_s);
t663 = t639 * t641;
t638 = sin(phi1_s);
t662 = phi1D_s * t638;
t640 = cos(phi2_s);
t661 = phi1D_s * t640;
t660 = phi1D_s * t641;
t637 = sin(phi2_s);
t659 = phi2D_s * t637;
t658 = phi2D_s * t640;
t657 = phi2D_s * t641;
t636 = sin(phi3_s);
t656 = phi3D_s * t636;
t655 = phi3D_s * t637;
t654 = phi3D_s * t640;
t653 = t639 * t659;
t652 = t639 * t655;
t651 = t638 * t659;
t650 = t638 * t658;
t649 = t637 * t657;
t648 = t640 * t657;
t647 = phi1D_s + t654;
t646 = phi3D_s + t661;
t645 = t647 * t636;
t644 = t637 * t660 + t650;
t643 = t637 * t662 - t648;
t642 = t646 * t638 + t649;
t635 = t642 * t639 + t641 * t645;
t634 = t647 * t639 * t638 + (t646 * t641 - t651) * t636;
t633 = -t646 * t663 + (t645 + t653) * t638;
t632 = t642 * t636 - t647 * t663;
t1 = [0 -t658 0; -t643 t640 * t660 - t651 0; t644 t638 * t661 + t649 0; 0 -t636 * t659 + t639 * t654 -t636 * t655 + t639 * t658; t632 t644 * t636 + t638 * t652 t633; -t634 t643 * t636 - t641 * t652 -t635; 0 -t636 * t654 - t653 -t636 * t658 - t652; t635 t639 * t650 + (-t638 * t656 + t639 * t660) * t637 t634; t633 -t639 * t648 + (t639 * t662 + t641 * t656) * t637 t632;];
GradMatD = t1;
case 2 % xyz
% aus codeexport/rotmatD_diff_eulxyz_matlab.m (euler_angle_calculations.mw)
t673 = cos(phi1_s);
t669 = sin(phi2_s);
t678 = phi1D_s * t669 + phi3D_s;
t694 = t678 * t673;
t670 = sin(phi1_s);
t672 = cos(phi2_s);
t688 = phi2D_s * t673;
t680 = t672 * t688;
t693 = t678 * t670 - t680;
t692 = phi1D_s * t670;
t691 = phi1D_s * t673;
t690 = phi2D_s * t669;
t689 = phi2D_s * t672;
t687 = phi3D_s * t669;
t686 = phi3D_s * t672;
t685 = t672 * t691;
t671 = cos(phi3_s);
t684 = t671 * t689;
t683 = t670 * t689;
t668 = sin(phi3_s);
t682 = t668 * t686;
t681 = t671 * t686;
t679 = -phi1D_s - t687;
t677 = t679 * t673;
t676 = t670 * t690 - t685;
t675 = t669 * t688 + t672 * t692;
t674 = t671 * t690 + t682;
t667 = t671 * t694 + (t679 * t668 + t684) * t670;
t666 = t679 * t671 * t670 + (-t683 - t694) * t668;
t665 = t668 * t677 - t693 * t671;
t664 = t693 * t668 + t671 * t677;
t1 = [0 t668 * t687 - t684 t668 * t690 - t681; t665 -t674 * t670 + t671 * t685 t666; t667 t675 * t671 + t673 * t682 -t664; 0 t668 * t689 + t671 * t687 t674; t664 t676 * t668 - t670 * t681 -t667; t666 -t675 * t668 + t673 * t681 t665; 0 -t690 0; t675 t669 * t691 + t683 0; t676 t669 * t692 - t680 0;];
GradMatD = t1;
case 3 % xzx
% aus codeexport/rotmatD_diff_eulxzx_matlab.m (euler_angle_calculations.mw)
t702 = cos(phi3_s);
t704 = cos(phi1_s);
t726 = t702 * t704;
t701 = sin(phi1_s);
t725 = phi1D_s * t701;
t703 = cos(phi2_s);
t724 = phi1D_s * t703;
t723 = phi1D_s * t704;
t700 = sin(phi2_s);
t722 = phi2D_s * t700;
t721 = phi2D_s * t703;
t720 = phi2D_s * t704;
t699 = sin(phi3_s);
t719 = phi3D_s * t699;
t718 = phi3D_s * t700;
t717 = phi3D_s * t703;
t716 = t702 * t722;
t715 = t702 * t718;
t714 = t701 * t722;
t713 = t701 * t721;
t712 = t700 * t720;
t711 = t703 * t720;
t710 = phi1D_s + t717;
t709 = phi3D_s + t724;
t708 = t710 * t699;
t707 = t700 * t723 + t713;
t706 = -t700 * t725 + t711;
t705 = t709 * t701 + t712;
t698 = t705 * t702 + t704 * t708;
t697 = t710 * t702 * t701 + (t709 * t704 - t714) * t699;
t696 = -t709 * t726 + (t708 + t716) * t701;
t695 = t705 * t699 - t710 * t726;
t1 = [0 -t721 0; -t707 -t701 * t724 - t712 0; t706 t703 * t723 - t714 0; 0 t699 * t717 + t716 t699 * t721 + t715; t696 -t702 * t711 + (t702 * t725 + t704 * t719) * t700 t695; -t698 -t702 * t713 + (t701 * t719 - t702 * t723) * t700 -t697; 0 -t699 * t722 + t702 * t717 -t699 * t718 + t702 * t721; t697 t706 * t699 + t704 * t715 t698; t695 t707 * t699 + t701 * t715 t696;];
GradMatD = t1;
case 4 % xzy
% aus codeexport/rotmatD_diff_eulxzy_matlab.m (euler_angle_calculations.mw)
t736 = cos(phi1_s);
t732 = sin(phi2_s);
t741 = phi1D_s * t732 - phi3D_s;
t757 = t741 * t736;
t733 = sin(phi1_s);
t735 = cos(phi2_s);
t751 = phi2D_s * t736;
t743 = t735 * t751;
t756 = t741 * t733 - t743;
t755 = phi1D_s * t733;
t754 = phi1D_s * t736;
t753 = phi2D_s * t732;
t752 = phi2D_s * t735;
t750 = phi3D_s * t732;
t749 = phi3D_s * t735;
t748 = t735 * t754;
t734 = cos(phi3_s);
t747 = t734 * t752;
t746 = t733 * t752;
t731 = sin(phi3_s);
t745 = t731 * t749;
t744 = t734 * t749;
t742 = phi1D_s - t750;
t740 = t742 * t736;
t739 = -t733 * t753 + t748;
t738 = -t732 * t751 - t735 * t755;
t737 = -t734 * t753 - t745;
t730 = t734 * t757 + (t742 * t731 + t747) * t733;
t729 = t742 * t734 * t733 + (-t746 - t757) * t731;
t728 = t731 * t740 - t756 * t734;
t727 = t756 * t731 + t734 * t740;
t1 = [0 t731 * t750 - t747 t731 * t753 - t744; -t730 t738 * t734 - t736 * t745 t727; t728 t737 * t733 + t734 * t748 t729; 0 t753 0; -t739 t732 * t755 - t743 0; t738 -t732 * t754 - t746 0; 0 -t731 * t752 - t734 * t750 t737; t729 t738 * t731 + t736 * t744 t728; -t727 t739 * t731 + t733 * t744 t730;];
GradMatD = t1;
case 5 % yxy
% aus codeexport/rotmatD_diff_eulyxy_matlab.m (euler_angle_calculations.mw)
t765 = cos(phi3_s);
t767 = cos(phi1_s);
t789 = t765 * t767;
t764 = sin(phi1_s);
t788 = phi1D_s * t764;
t766 = cos(phi2_s);
t787 = phi1D_s * t766;
t786 = phi1D_s * t767;
t763 = sin(phi2_s);
t785 = phi2D_s * t763;
t784 = phi2D_s * t766;
t783 = phi2D_s * t767;
t762 = sin(phi3_s);
t782 = phi3D_s * t762;
t781 = phi3D_s * t763;
t780 = phi3D_s * t766;
t779 = t765 * t785;
t778 = t765 * t781;
t777 = t764 * t785;
t776 = t764 * t784;
t775 = t763 * t783;
t774 = t766 * t783;
t773 = phi1D_s + t780;
t772 = phi3D_s + t787;
t771 = t773 * t762;
t770 = t763 * t786 + t776;
t769 = -t763 * t788 + t774;
t768 = t772 * t764 + t775;
t761 = t768 * t765 + t767 * t771;
t760 = t773 * t765 * t764 + (t772 * t767 - t777) * t762;
t759 = -t772 * t789 + (t771 + t779) * t764;
t758 = t768 * t762 - t773 * t789;
t1 = [t758 t770 * t762 + t764 * t778 t759; 0 -t762 * t785 + t765 * t780 -t762 * t781 + t765 * t784; t760 t769 * t762 + t767 * t778 t761; t769 t766 * t786 - t777 0; 0 -t784 0; -t770 -t764 * t787 - t775 0; -t761 -t765 * t776 + (t764 * t782 - t765 * t786) * t763 -t760; 0 t762 * t780 + t779 t762 * t784 + t778; t759 -t765 * t774 + (t765 * t788 + t767 * t782) * t763 t758;];
GradMatD = t1;
case 6 % yxz
% aus codeexport/rotmatD_diff_eulyxz_matlab.m (euler_angle_calculations.mw)
t799 = cos(phi1_s);
t795 = sin(phi2_s);
t804 = phi1D_s * t795 - phi3D_s;
t820 = t804 * t799;
t796 = sin(phi1_s);
t798 = cos(phi2_s);
t814 = phi2D_s * t799;
t806 = t798 * t814;
t819 = t804 * t796 - t806;
t818 = phi1D_s * t796;
t817 = phi1D_s * t799;
t816 = phi2D_s * t795;
t815 = phi2D_s * t798;
t813 = phi3D_s * t795;
t812 = phi3D_s * t798;
t811 = t798 * t817;
t797 = cos(phi3_s);
t810 = t797 * t815;
t809 = t796 * t815;
t794 = sin(phi3_s);
t808 = t794 * t812;
t807 = t797 * t812;
t805 = phi1D_s - t813;
t803 = t805 * t799;
t802 = -t796 * t816 + t811;
t801 = -t795 * t814 - t798 * t818;
t800 = -t797 * t816 - t808;
t793 = t797 * t820 + (t805 * t794 + t810) * t796;
t792 = t805 * t797 * t796 + (-t809 - t820) * t794;
t791 = t794 * t803 - t819 * t797;
t790 = t819 * t794 + t797 * t803;
t1 = [-t790 t802 * t794 + t796 * t807 t793; 0 -t794 * t815 - t797 * t813 t800; t792 t801 * t794 + t799 * t807 t791; t791 t800 * t796 + t797 * t811 t792; 0 t794 * t813 - t810 t794 * t816 - t807; -t793 t801 * t797 - t799 * t808 t790; t801 -t795 * t817 - t809 0; 0 t816 0; -t802 t795 * t818 - t806 0;];
GradMatD = t1;
case 7 % yzx
% aus codeexport/rotmatD_diff_eulyzx_matlab.m (euler_angle_calculations.mw)
t830 = cos(phi1_s);
t826 = sin(phi2_s);
t835 = phi1D_s * t826 + phi3D_s;
t851 = t835 * t830;
t827 = sin(phi1_s);
t829 = cos(phi2_s);
t845 = phi2D_s * t830;
t837 = t829 * t845;
t850 = t835 * t827 - t837;
t849 = phi1D_s * t827;
t848 = phi1D_s * t830;
t847 = phi2D_s * t826;
t846 = phi2D_s * t829;
t844 = phi3D_s * t826;
t843 = phi3D_s * t829;
t842 = t829 * t848;
t828 = cos(phi3_s);
t841 = t828 * t846;
t840 = t827 * t846;
t825 = sin(phi3_s);
t839 = t825 * t843;
t838 = t828 * t843;
t836 = -phi1D_s - t844;
t834 = t836 * t830;
t833 = t827 * t847 - t842;
t832 = t826 * t845 + t829 * t849;
t831 = t828 * t847 + t839;
t824 = t828 * t851 + (t836 * t825 + t841) * t827;
t823 = t836 * t828 * t827 + (-t840 - t851) * t825;
t822 = t825 * t834 - t850 * t828;
t821 = t850 * t825 + t828 * t834;
t1 = [t833 t826 * t849 - t837 0; 0 -t847 0; t832 t826 * t848 + t840 0; t824 t832 * t828 + t830 * t839 -t821; 0 t825 * t844 - t841 t825 * t847 - t838; t822 -t831 * t827 + t828 * t842 t823; t823 -t832 * t825 + t830 * t838 t822; 0 t825 * t846 + t828 * t844 t831; t821 t833 * t825 - t827 * t838 -t824;];
GradMatD = t1;
case 8 % yzy
% aus codeexport/rotmatD_diff_eulyzy_matlab.m (euler_angle_calculations.mw)
t859 = cos(phi3_s);
t861 = cos(phi1_s);
t883 = t859 * t861;
t858 = sin(phi1_s);
t882 = phi1D_s * t858;
t860 = cos(phi2_s);
t881 = phi1D_s * t860;
t880 = phi1D_s * t861;
t857 = sin(phi2_s);
t879 = phi2D_s * t857;
t878 = phi2D_s * t860;
t877 = phi2D_s * t861;
t856 = sin(phi3_s);
t876 = phi3D_s * t856;
t875 = phi3D_s * t857;
t874 = phi3D_s * t860;
t873 = t859 * t879;
t872 = t859 * t875;
t871 = t858 * t879;
t870 = t858 * t878;
t869 = t857 * t877;
t868 = t860 * t877;
t867 = phi1D_s + t874;
t866 = phi3D_s + t881;
t865 = t867 * t856;
t864 = t857 * t880 + t870;
t863 = t857 * t882 - t868;
t862 = t866 * t858 + t869;
t855 = t862 * t859 + t861 * t865;
t854 = t867 * t859 * t858 + (t866 * t861 - t871) * t856;
t853 = -t866 * t883 + (t865 + t873) * t858;
t852 = t862 * t856 - t867 * t883;
t1 = [t853 -t859 * t868 + (t859 * t882 + t861 * t876) * t857 t852; 0 -t856 * t874 - t873 -t856 * t878 - t872; t855 t859 * t870 + (-t858 * t876 + t859 * t880) * t857 t854; t864 t858 * t881 + t869 0; 0 -t878 0; -t863 t860 * t880 - t871 0; -t854 t863 * t856 - t861 * t872 -t855; 0 -t856 * t879 + t859 * t874 -t856 * t875 + t859 * t878; t852 t864 * t856 + t858 * t872 t853;];
GradMatD = t1;
case 9 % zxy
% aus codeexport/rotmatD_diff_eulzxy_matlab.m (euler_angle_calculations.mw)
t893 = cos(phi1_s);
t889 = sin(phi2_s);
t898 = phi1D_s * t889 + phi3D_s;
t914 = t898 * t893;
t890 = sin(phi1_s);
t892 = cos(phi2_s);
t908 = phi2D_s * t893;
t900 = t892 * t908;
t913 = t898 * t890 - t900;
t912 = phi1D_s * t890;
t911 = phi1D_s * t893;
t910 = phi2D_s * t889;
t909 = phi2D_s * t892;
t907 = phi3D_s * t889;
t906 = phi3D_s * t892;
t905 = t892 * t911;
t891 = cos(phi3_s);
t904 = t891 * t909;
t903 = t890 * t909;
t888 = sin(phi3_s);
t902 = t888 * t906;
t901 = t891 * t906;
t899 = -phi1D_s - t907;
t897 = t899 * t893;
t896 = t890 * t910 - t905;
t895 = t889 * t908 + t892 * t912;
t894 = t891 * t910 + t902;
t887 = t891 * t914 + (t899 * t888 + t904) * t890;
t886 = t899 * t891 * t890 + (-t903 - t914) * t888;
t885 = t888 * t897 - t913 * t891;
t884 = t913 * t888 + t891 * t897;
t1 = [t884 t896 * t888 - t890 * t901 -t887; t886 -t895 * t888 + t893 * t901 t885; 0 t888 * t909 + t891 * t907 t894; t895 t889 * t911 + t903 0; t896 t889 * t912 - t900 0; 0 -t910 0; t885 -t894 * t890 + t891 * t905 t886; t887 t895 * t891 + t893 * t902 -t884; 0 t888 * t907 - t904 t888 * t910 - t901;];
GradMatD = t1;
case 10 % zxz
% aus codeexport/rotmatD_diff_eulzxz_matlab.m (euler_angle_calculations.mw)
t922 = cos(phi3_s);
t924 = cos(phi1_s);
t946 = t922 * t924;
t921 = sin(phi1_s);
t945 = phi1D_s * t921;
t923 = cos(phi2_s);
t944 = phi1D_s * t923;
t943 = phi1D_s * t924;
t920 = sin(phi2_s);
t942 = phi2D_s * t920;
t941 = phi2D_s * t923;
t940 = phi2D_s * t924;
t919 = sin(phi3_s);
t939 = phi3D_s * t919;
t938 = phi3D_s * t920;
t937 = phi3D_s * t923;
t936 = t922 * t942;
t935 = t922 * t938;
t934 = t921 * t942;
t933 = t921 * t941;
t932 = t920 * t940;
t931 = t923 * t940;
t930 = phi1D_s + t937;
t929 = phi3D_s + t944;
t928 = t930 * t919;
t927 = t920 * t943 + t933;
t926 = t920 * t945 - t931;
t925 = t929 * t921 + t932;
t918 = t925 * t922 + t924 * t928;
t917 = t930 * t922 * t921 + (t929 * t924 - t934) * t919;
t916 = -t929 * t946 + (t928 + t936) * t921;
t915 = t925 * t919 - t930 * t946;
t1 = [t915 t927 * t919 + t921 * t935 t916; -t917 t926 * t919 - t924 * t935 -t918; 0 -t919 * t942 + t922 * t937 -t919 * t938 + t922 * t941; t918 t922 * t933 + (-t921 * t939 + t922 * t943) * t920 t917; t916 -t922 * t931 + (t922 * t945 + t924 * t939) * t920 t915; 0 -t919 * t937 - t936 -t919 * t941 - t935; -t926 t923 * t943 - t934 0; t927 t921 * t944 + t932 0; 0 -t941 0;];
GradMatD = t1;
case 11 % zyx
% aus codeexport/rotmatD_diff_eulzyx_matlab.m (euler_angle_calculations.mw)
t956 = cos(phi1_s);
t952 = sin(phi2_s);
t961 = phi1D_s * t952 - phi3D_s;
t977 = t961 * t956;
t953 = sin(phi1_s);
t955 = cos(phi2_s);
t971 = phi2D_s * t956;
t963 = t955 * t971;
t976 = t961 * t953 - t963;
t975 = phi1D_s * t953;
t974 = phi1D_s * t956;
t973 = phi2D_s * t952;
t972 = phi2D_s * t955;
t970 = phi3D_s * t952;
t969 = phi3D_s * t955;
t968 = t955 * t974;
t954 = cos(phi3_s);
t967 = t954 * t972;
t966 = t953 * t972;
t951 = sin(phi3_s);
t965 = t951 * t969;
t964 = t954 * t969;
t962 = phi1D_s - t970;
t960 = t962 * t956;
t959 = -t953 * t973 + t968;
t958 = -t952 * t971 - t955 * t975;
t957 = -t954 * t973 - t965;
t950 = t954 * t977 + (t962 * t951 + t967) * t953;
t949 = t962 * t954 * t953 + (-t966 - t977) * t951;
t948 = t951 * t960 - t976 * t954;
t947 = t976 * t951 + t954 * t960;
t1 = [-t959 t952 * t975 - t963 0; t958 -t952 * t974 - t966 0; 0 t973 0; t949 t958 * t951 + t956 * t964 t948; -t947 t959 * t951 + t953 * t964 t950; 0 -t951 * t972 - t954 * t970 t957; -t950 t958 * t954 - t956 * t965 t947; t948 t957 * t953 + t954 * t968 t949; 0 t951 * t970 - t967 t951 * t973 - t964;];
GradMatD = t1;
case 12 % zyz
% aus codeexport/rotmatD_diff_eulzyz_matlab.m (euler_angle_calculations.mw)
t985 = cos(phi3_s);
t987 = cos(phi1_s);
t1009 = t985 * t987;
t984 = sin(phi1_s);
t1008 = phi1D_s * t984;
t986 = cos(phi2_s);
t1007 = phi1D_s * t986;
t1006 = phi1D_s * t987;
t983 = sin(phi2_s);
t1005 = phi2D_s * t983;
t1004 = phi2D_s * t986;
t1003 = phi2D_s * t987;
t982 = sin(phi3_s);
t1002 = phi3D_s * t982;
t1001 = phi3D_s * t983;
t1000 = phi3D_s * t986;
t999 = t985 * t1005;
t998 = t985 * t1001;
t997 = t984 * t1005;
t996 = t984 * t1004;
t995 = t983 * t1003;
t994 = t986 * t1003;
t993 = phi1D_s + t1000;
t992 = phi3D_s + t1007;
t991 = t993 * t982;
t990 = t983 * t1006 + t996;
t989 = -t983 * t1008 + t994;
t988 = t992 * t984 + t995;
t981 = t988 * t985 + t987 * t991;
t980 = t993 * t985 * t984 + (t992 * t987 - t997) * t982;
t979 = -t992 * t1009 + (t991 + t999) * t984;
t978 = -t993 * t1009 + t988 * t982;
t1 = [t979 -t985 * t994 + (t987 * t1002 + t985 * t1008) * t983 t978; -t981 -t985 * t996 + (t984 * t1002 - t985 * t1006) * t983 -t980; 0 t982 * t1000 + t999 t982 * t1004 + t998; t980 t989 * t982 + t987 * t998 t981; t978 t990 * t982 + t984 * t998 t979; 0 t985 * t1000 - t982 * t1005 -t982 * t1001 + t985 * t1004; -t990 -t984 * t1007 - t995 0; t989 t986 * t1006 - t997 0; 0 -t1004 0;];
GradMatD = t1;
otherwise
error('rotmatD_diff_eul: conv has to be 1 to 12');
end
