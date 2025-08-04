function quat_out = conv_quat(quat_in)

% Conversion of quaternion in same representation
    if quat_in(1) < 0
        quat_out = -quat_in;
    elseif  quat_in(1) == 0
        if quat_in(2) < 0
            quat_out = -quat_in;
        elseif quat_in(2) == 0
            if quat_in(3) < 0
                quat_out = -quat_in;
            elseif quat_in(3) == 0
                if quat_in(4) < 0
                    quat_out = -quat_in;
                else
                    quat_out = quat_in;
                end
            else
                quat_out = quat_in;
            end
        else
            quat_out = quat_in;
        end
        else
            quat_out = quat_in;
    end
end