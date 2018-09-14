function limitPlot(filename, suffix, heading, xtxt,x, ytxt,y, lowerBound, upperBound)
%
% Blows up and plots each region within input plot which exceeds bounds.
%

j = 1;          % index into current plot
idx = 0;        % plot count 
mode = 0;       % -1 for below lower bound, 0 at start or end or inside bounds, +1 above upper bound.
newMode = NaN;
len = length(y) + 1;
deltaX = x(2)-x(1);

for i=1:len
    if (i == len)
        newMode = 0;
    elseif (y(i) > upperBound)
        newMode = 1;
    elseif (y(i) < lowerBound)
        newMode = -1;
    else
        newMode = 0;
    end

    if (mode ~= 0 && mode ~= newMode)
        name = sprintf('%s_%s_%03d', filename, suffix, idx);
        mainTitle = sprintf('Path_{%s}  %s', suffix, heading);
        idx = idx + 1;
        doPlot(name, mainTitle, xtxt,x2, ytxt,y2, ...
               lowerBound, upperBound, deltaX, mode)
        clear x2;
        clear y2;
        j = 1;
    end
    if newMode ~= 0
        x2(j) = x(i);
        y2(j) = y(i);
        j = j + 1;
    end
    mode = newMode;
end
end

function doPlot(name, mainTitle, xtxt,x, ytxt,y, lowerBound, upperBound, ...
                deltaX, boundDirection)
peakThresh = 0.002;              
areaThresh = 0.001;

bound = NaN;
yn = NaN;
yp = NaN;
if (boundDirection == 1)
    bound = upperBound;
    yn = min(y);
    yp = max(y);
else
    bound = lowerBound;
    yn = max(y);
    yp = min(y);
end    

boundPeak = yp - bound;

boundArea = deltaX * trapz(y - bound);

xBoundRange = deltaX * length(x);
yBoundRange = (upperBound - lowerBound) / 2;
peakPercent = boundPeak / yBoundRange;
areaPercent = boundArea / xBoundRange / yBoundRange;

if (abs(peakPercent) > peakThresh || abs(areaPercent) > areaThresh)
    figure('visible','off');
    h = area(x,y);
    set(h, 'BaseValue', bound)
    grid on
    title({mainTitle}, 'fontsize', 24)
    xlabel({xtxt})
    ylabel({ytxt})

    orient landscape
    print('-dpdf', name)
    
    disp(sprintf('  %s\t  X:%2.2f:%2.2f %s\t  Y:%+2.2f:%+2.2f %s\t\t Peak:%+1.2f%%\t  Avg:%+1.2f%%', ...
                 name, x(1), x(end), xtxt, yn, yp, ytxt, 100*peakPercent, ...
                 100*areaPercent))
end

end
