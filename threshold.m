case 'threshold'
        if nIn<2, varargout{1} = info(id).threshold; return; end
        if nargout, varargout{1} = info(id).threshold; end
        if v<5, RTBoxWarn('notSupported', in1, 5); return; end
        if isempty(in2), in2 = 1; end
        in2 = round(in2);
        in2 = max(min(in2, 4), 1);
        info(id).threshold = in2;
        b = sum(bitset(0, [1 2 4 7], [info(id).TTLresting bitget(in2-1, 1:2)]));
        writeEEPROM(s, 225, uint8(b));