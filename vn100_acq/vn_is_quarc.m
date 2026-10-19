function tf = vn_is_quarc(mdl)
%VN_IS_QUARC  True if the model's active configuration uses a QUARC target (quarc_*.tlc).
tf = strncmpi(get_param(mdl, 'SystemTargetFile'), 'quarc', 5);
end
