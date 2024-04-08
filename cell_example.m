function cell_format = cell_example(motor1_fail,motor2_fail)
    
%row = motor1_fail
%col = motor2_fail
%motor1_fail = 4000;
%motor2_fail = 4000;

cell_format = cell(motor1_fail,motor2_fail);

    for x= 1:motor1_fail   %cell is start 1.   not 0
        for y= 1:motor2_fail   %cell is start 1.   not 0
            mat = [x, y];
            cell_format{x,y} = mat;
        end
    end

end

