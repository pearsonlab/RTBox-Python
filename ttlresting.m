case 'ttlresting'
        if nIn<2, varargout{1} = info(id).TTLresting; return; end
        if nargout, varargout{1} = info(id).TTLresting; end
        if v<3.1, RTBoxWarn('notSupported', in1, 3.1); return; end
        if isempty(in2), in2 = logical([0 1]); end
        info(id).TTLresting = in2;
        if v<4.4
            b8 = get8bytes(s);
            b8(3) = in2(1)*240; % '11110000'
            set8bytes(s, b8);
        else
            if numel(in2)>2, in2 = in2(1:2);
            elseif numel(in2)<2, in2(2) = info(id).TTLresting(2);
            end
            b = bitget(info(id).threshold-1, 1:2); % threshold bits
            b = sum(bitset(0, [1 2 4 7], [in2 b]));
            writeEEPROM(s, 225, uint8(b));
        end