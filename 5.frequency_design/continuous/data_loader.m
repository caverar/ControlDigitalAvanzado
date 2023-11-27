classdef data_loader
    methods(Static)
        function [out,ctr] = square_signal_loader(file_name, freq, ...
                signal_period, min_value, max_value)
            % load file
            file = readtable(file_name);
            ref = file.r;
            omega = file.omega;
            u = file.u;
            % Order data
            zero_index = 0;
            i = 2;
            while i<length(ref) && zero_index == 0
                if ref(i-1)==min_value && ref(i)==max_value
                    zero_index=i;
                end
                i=i+1;            
            end
            begin_index = zero_index-(freq*signal_period/2);
            end_index = (zero_index-(freq*signal_period/2)-1) + ...
                (freq*signal_period*2);
            out=omega(begin_index: end_index +1);
            ctr=u(begin_index: end_index +1);
        end
        function [out,ctr] = triangular_signal_loader(file_name, freq, ...
                signal_period, min_value, max_value)
            % load file
            file = readtable(file_name);
            ref = file.r;
            omega = file.omega;
            u = file.u;
            % Order data
            zero_index = 0;
            i = 1;
            while i<length(ref) && zero_index == 0
                if ref(i)==min_value
                    zero_index=i;
                end
                i=i+1;            
            end
            begin_index = zero_index;
            end_index = zero_index + (freq*signal_period*2);
            out=omega(begin_index: end_index);
            ctr=u(begin_index: end_index);
        end 
    end
end