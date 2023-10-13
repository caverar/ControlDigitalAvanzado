classdef w_transform
    methods (Static)
        function tf_w = z2w(tf_z, Ts)
            [num,den] = tfdata(tf_z);
            num = cell2mat(num);
            den = cell2mat(den);
            syms w z;
            tf_z_sym = poly2sym(num,z)/poly2sym(den,z);
            w_subs= ((1+(w*sym(Ts)/2)) / (1-(w*sym(Ts)/2)));
            tf_w_sym = subs(tf_z_sym,z,w_subs);
            [num,den] = numden(tf_w_sym);
            tf_w = tf(sym2poly(num),sym2poly(den));
        end
        function tf_z = w2z(tf_w, Ts)
            [num,den] = tfdata(tf_w);
            num = cell2mat(num);
            den = cell2mat(den);
            syms w z;
            tf_w_sym = poly2sym(num,w)/poly2sym(den,w);
            z_subs = (2/sym(Ts))*((z-1)/(z+1));
            tf_z_sym = subs(tf_w_sym,w,z_subs);
            [num,den] = numden(tf_z_sym);
            tf_z = tf(sym2poly(num),sym2poly(den),Ts);
        end
    end
end
