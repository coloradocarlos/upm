%module pyupm_hm11
%include "../upm.i"
%include "carrays.i"

%feature("autodoc", "3");

%{
    #include "hm11.h"
    speed_t int_B9600 = B9600;
%}
%include "hm11.h"
speed_t int_B9600 = B9600;
%array_class(char, charArray);
