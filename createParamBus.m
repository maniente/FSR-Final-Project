function createParamBus()
    % Define individual elements of the param struct
    elems = [];

    elems(end+1) = Simulink.BusElement;
    elems(end).Name = 'ut';
    elems(end).Dimensions = 1;
    elems(end).DataType = 'double';

    elems(end+1) = Simulink.BusElement;
    elems(end).Name = 'un';
    elems(end).Dimensions = 1;
    elems(end).DataType = 'double';

    elems(end+1) = Simulink.BusElement;
    elems(end).Name = 'ct';
    elems(end).Dimensions = 1;
    elems(end).DataType = 'double';

    % (Repeat for all fields in your `param` struct)
    % e.g. alphaA, omega, delta, kp, kd, l, diameter, diameterI, m, g, etc.

    % Create the bus object
    ParamBus = Simulink.Bus;
    ParamBus.Elements = elems;

    % Save it to workspace
    assignin('base', 'ParamBus', ParamBus);
end
