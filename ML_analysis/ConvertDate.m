function date = ConvertDate(seconds, nanos)
date = datestr((seconds + 8*3600 + nanos / 1e9)/86400 + datenum(1970,1,1), 'yyyy-mm-dd HH:MM:SS:FFF');
end

