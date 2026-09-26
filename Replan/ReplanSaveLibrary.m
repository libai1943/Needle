function ReplanSaveLibrary(filename, lib)
    for j = 1:numel(lib.entries)
        if isfield(lib.entries{j}, 'model')
            lib.entries{j} = rmfield(lib.entries{j}, 'model');
        end
        if isfield(lib.entries{j}, 'corrector')
            lib.entries{j} = rmfield(lib.entries{j}, 'corrector');
        end
        if isfield(lib.entries{j}, 'fallback_model')
            lib.entries{j} = rmfield(lib.entries{j}, 'fallback_model');
        end
    end
    save(filename, 'lib', '-v7.3');
end
