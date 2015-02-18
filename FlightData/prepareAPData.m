function prepareAPData (filename)
    [~, name, ext] = fileparts(filename);
    
    % Load the file, drop NANs, and find the end of the header section
    eval(['perl noNansTelem.pl <' filename '>temp.csv']);
    offset = findHeaderEnd(filename);

    % Read the csv file, and save into MAT-file
    data = csvread('temp.csv', offset, 0);
    outFilename = strcat(name,'.mat');
    save(outFilename, 'data');

    % Delete the temporary file
    delete('temp.csv');

end % function prepareAPData()

function [line] = findHeaderEnd(filename)
    fid = fopen(filename);
    tline = fgets(fid);
    line = 1;
    while ischar(tline)
        %disp(tline)
        k = strfind(tline, 'End description data');
        if k
            break;
        end
        line = line + 1;
        tline = fgets(fid);
    end
    
    %fprintf('Found end of header section at %i.\n', line);
    line = line + 3; % account for space after header
end