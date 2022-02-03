function [y_data_1, y_data_2] = get_cartesian_error_data(data_1, data_2, t_start, t_step, t_end)

[p_med_1, dp_med_1, ddp_med_1, p_ref_1, dp_ref_1, ddp_ref_1, p_e_1, dp_e_1, ddp_e_1] = get_cartesian_position_data( data_1, t_start, t_step, t_end);
[rpy_med_1, rpy_ref_1, rpy_e_1] = get_cartesian_orientation_data(data_1, t_start, t_step, t_end);

[p_med_2, dp_med_2, ddp_med_2, p_ref_2, dp_ref_2, ddp_ref_2, p_e_2, dp_e_2, ddp_e_2] = get_cartesian_position_data( data_2, t_start, t_step, t_end);
[rpy_med_2, rpy_ref_2, rpy_e_2] = get_cartesian_orientation_data(data_2, t_start, t_step, t_end);



y_data_1 = [p_e_1(:,1)';p_e_1(:,2)';p_e_1(:,1)' ; ddp_e_1(:,1)';ddp_e_1(:,2)';ddp_e_1(:,3)'; rpy_e_1(:,1)'; rpy_e_1(:,2)'; rpy_e_1(:,3)'];
y_data_2 = [p_e_2(:,1)';p_e_2(:,2)';p_e_2(:,1)' ; ddp_e_2(:,1)';ddp_e_2(:,2)';ddp_e_2(:,3)'; rpy_e_2(:,1)'; rpy_e_2(:,2)'; rpy_e_2(:,3)'];

end