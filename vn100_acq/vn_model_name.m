function name = vn_model_name(mdl)
%VN_MODEL_NAME  Load a model given by name or by file path; return its block-diagram name.
%   name = vn_model_name('exo_df')
%   name = vn_model_name('C:\...\exo_df_simulink\exo_df')       % extension optional
%   name = vn_model_name('C:\...\exo_df_simulink\exo_df.slx')
%   load_system accepts a path, but get_param, add_block and set_param need
%   the block-diagram name, which is the file name without folder or
%   extension.
[folder, name, ext] = fileparts(mdl);
if isempty(folder) && isempty(ext)
    load_system(name);                        % plain name, resolved on the MATLAB path
    return;
end
if isempty(ext)
    if exist(fullfile(folder, [name '.slx']), 'file')
        ext = '.slx';
    elseif exist(fullfile(folder, [name '.mdl']), 'file')
        ext = '.mdl';
    else
        error('vn_model_name:nofile', 'No %s.slx or %s.mdl in %s.', name, name, folder);
    end
end
f = fullfile(folder, [name ext]);
if ~exist(f, 'file')
    error('vn_model_name:nofile', 'Model file not found: %s', f);
end
if bdIsLoaded(name)
    loaded = get_param(name, 'FileName');
    if ~strcmpi(loaded, f)
        error('vn_model_name:shadow', ['A different model named ''%s'' is already loaded (%s). ' ...
            'Close it with close_system(''%s'') first.'], name, loaded, name);
    end
else
    load_system(f);
end
end
