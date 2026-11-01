modelName = 'DiffDrive_Interactive';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% 1. Set model to run forever at real-world speed
set_param(modelName, 'StopTime', 'inf');
set_param(modelName, 'EnablePacing', 'on', 'PacingRate', '1');

assignin('base', 'R', 0.05); 
assignin('base', 'L', 0.20); 

% 2. Place Blocks
add_block('simulink/Sources/Constant', [modelName '/Omega_Left'], 'Position', [20, 90, 60, 120], 'Value', '5');
add_block('simulink/Sources/Constant', [modelName '/Omega_Right'], 'Position', [20, 390, 60, 420], 'Value', '3');

add_block('simulink/Math Operations/Gain', [modelName '/Radius_Left'], 'Position', [120, 90, 160, 120], 'Gain', 'R');
add_block('simulink/Math Operations/Gain', [modelName '/Radius_Right'], 'Position', [120, 390, 160, 420], 'Gain', 'R');

add_block('simulink/Math Operations/Sum', [modelName '/Sum_Linear'], 'Position', [250, 190, 280, 220], 'Inputs', '++');
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Angular'], 'Position', [250, 290, 280, 320], 'Inputs', '-+');

add_block('simulink/Math Operations/Gain', [modelName '/Gain_v'], 'Position', [340, 190, 380, 220], 'Gain', '0.5');
add_block('simulink/Math Operations/Gain', [modelName '/Gain_w'], 'Position', [340, 290, 380, 320], 'Gain', '1/L');

add_block('simulink/Continuous/Integrator', [modelName '/Int_Theta'], 'Position', [450, 290, 480, 320]);

add_block('simulink/Math Operations/Trigonometric Function', [modelName '/Cos_Theta'], 'Position', [550, 140, 590, 170], 'Operator', 'cos');
add_block('simulink/Math Operations/Trigonometric Function', [modelName '/Sin_Theta'], 'Position', [550, 290, 590, 320], 'Operator', 'sin');

add_block('simulink/Math Operations/Product', [modelName '/Prod_Xdot'], 'Position', [680, 145, 710, 185]);
add_block('simulink/Math Operations/Product', [modelName '/Prod_Ydot'], 'Position', [680, 295, 710, 335]);

add_block('simulink/Continuous/Integrator', [modelName '/Int_X'], 'Position', [780, 150, 810, 180]);
add_block('simulink/Continuous/Integrator', [modelName '/Int_Y'], 'Position', [780, 300, 810, 330]);

add_block('simulink/Signal Routing/Mux', [modelName '/Pose_Mux'], 'Position', [900, 280, 905, 380], 'Inputs', '3', 'DisplayOption', 'bar');

% Replaced XY Graph with a MATLAB Function block for the visualizer
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Visualizer'], 'Position', [950, 312, 1020, 348]);

% 3. Route Signals
connect = @(src, dst) add_line(modelName, src, dst, 'autorouting', 'smart');

connect('Omega_Left/1', 'Radius_Left/1');
connect('Omega_Right/1', 'Radius_Right/1');
connect('Radius_Left/1', 'Sum_Linear/1');
connect('Radius_Right/1', 'Sum_Linear/2');
connect('Radius_Left/1', 'Sum_Angular/1'); 
connect('Radius_Right/1', 'Sum_Angular/2'); 
connect('Sum_Linear/1', 'Gain_v/1');
connect('Sum_Angular/1', 'Gain_w/1');
connect('Gain_w/1', 'Int_Theta/1');
connect('Int_Theta/1', 'Cos_Theta/1');
connect('Int_Theta/1', 'Sin_Theta/1');
connect('Cos_Theta/1', 'Prod_Xdot/1');
connect('Gain_v/1', 'Prod_Xdot/2');
connect('Sin_Theta/1', 'Prod_Ydot/1');
connect('Gain_v/1', 'Prod_Ydot/2');
connect('Prod_Xdot/1', 'Int_X/1');
connect('Prod_Ydot/1', 'Int_Y/1');
connect('Int_X/1', 'Pose_Mux/1');
connect('Int_Y/1', 'Pose_Mux/2');
connect('Int_Theta/1', 'Pose_Mux/3');

% Feed the Mux directly into the Visualizer
connect('Pose_Mux/1', 'Visualizer/1');

disp('Interactive layout ready. Please update the Visualizer block as instructed.');