function [h] = figurename(name)
h = figure('name',name,'numbertitle','off');
if isa(h,'matlab.ui.Figure')
    h = h.Number;
end
