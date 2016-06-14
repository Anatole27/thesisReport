% Frequency weights generation
display('Creating frequency weights')
[Win,Wout,inputs,outputs] = weightsHinfStruct;

% Notch feedback
display('Notch architecture')
notchFeedbackOpt

% Strain feedback
display('Strain architecture')
strainFeedbackOpt

% Gyroscopes feedback
display('Gyro architecture')
gyrFeedbackOpt

% Accelerometers feedback
display('Acc architecture')
accFeedbackOpt