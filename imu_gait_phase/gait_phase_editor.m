function R = gait_phase_editor(t, gyro, S0, opt, name)
%GAIT_PHASE_EDITOR  Review and edit heel strikes and toe-offs for the continuous gait phase.
%   R = gait_phase_editor(t, gyro, S0, opt, name)
%     t     [n x 1] IMU time (t_sim) [s]       gyro  [n x 3] shoe angular rate [rad/s]
%     S0    start state: .axisChoice (1/2/3 = gyro x/y/z, 4 = principal axis),
%           .invert (true/false), .hsIdx (sample indices; [] = automatic detection),
%           .hsAutoIdx (optional, automatic set of a resumed file), .toIdx and
%           .toAutoIdx (optional toe-offs; [] = detected from the heel strikes)
%     opt   options of gait_phase_run (lpHz, strideRange, snapWin, minHsSep,
%           delTol, winWidth, useGui and the gait_hs_detect / gait_to_detect fields)
%     name  shown in the window title
%   With opt.useGui = false the automatic detection is returned directly.
%   Otherwise the window blocks until "Save and close" (R.cancelled = false)
%   or "Cancel" / closing the window (R.cancelled = true).
%   The gait phase comes from the heel strikes only; toe-offs never change it.
%
%   Window
%     left panel   reference signal (gyro x/y/z or principal axis), Invert,
%                  re-run automatic detection (both events) or toe-offs only,
%                  click mode, window width, navigation, undo, counts,
%                  Save and close / Cancel
%     top axes     reference signal with heel strikes (red circles) and
%                  toe-offs (yellow squares).  Strides outside the valid
%                  stride time are shaded red; valid strides without exactly
%                  one toe-off are shaded yellow.  Add modes: a click adds the
%                  event at the lowest point within opt.snapWin s.  Delete
%                  modes: a click removes the nearest event of that kind
%                  (within opt.delTol s).  Right-click (ctrl-click) deletes
%                  the nearest event of either kind in any mode.
%     middle axes  resulting gait phase [%], toe-offs marked at their phase
%     bottom axes  stride time over the whole trial (red = outside
%                  opt.strideRange x median); click to jump there
%   Keys: left/right arrow pan, mouse wheel pans, n = navigate,
%   a/d = add/delete heel strike, t/r = add/remove toe-off, u = undo,
%   s = next suspicious stride.

t    = t(:);
gyro = double(gyro);
n    = numel(t);
fs   = 1 / median(diff(t));
axisNames  = {'Gyro x', 'Gyro y', 'Gyro z', 'Principal axis (PCA)'};
winWidths  = [5 10 20 60 Inf];
winLabels  = {'5 s', '10 s', '20 s', '60 s', 'Whole trial'};
cHS = [0.85 0.1 0.1];
cTO = [0.95 0.7 0.1];

axisChoice = S0.axisChoice;
invert     = logical(S0.invert);
w = []; axVec = []; sigName = '';
hs = zeros(0, 1); hsAuto = zeros(0, 1); ms = zeros(0, 1);
to = zeros(0, 1); toAuto = zeros(0, 1);
hsT = zeros(0, 1); toT = zeros(0, 1); T = zeros(0, 1); okS = false(0, 1); ph = [];
nTo = zeros(0, 1); badHs = zeros(0, 1); badTo = zeros(0, 1); susp = zeros(0, 1);
undoStack  = {};
clickMode  = 'nav';
lastAction = '';
yl  = [-1 1];
tLo = t(1);
tHi = t(end);
winW = opt.winWidth;
winC = tLo + winW / 2;
R = struct('cancelled', true);
fig = []; popWin = [];
suggInv = invertSuggestion(axisChoice);

computeSignal();
if isempty(S0.hsIdx)
    runAuto();
else
    [hsAuto, ms] = gait_hs_detect(t, w, opt);
    if isfield(S0, 'hsAutoIdx') && ~isempty(S0.hsAutoIdx), hsAuto = unique(S0.hsAutoIdx(:)); end
    hs = unique(S0.hsIdx(:));
    if isfield(S0, 'toIdx') && ~isempty(S0.toIdx)
        to = unique(S0.toIdx(:));
        toAuto = to;
        if isfield(S0, 'toAutoIdx') && ~isempty(S0.toAutoIdx), toAuto = unique(S0.toAutoIdx(:)); end
    else
        runAutoTo();                          % file saved before toe-offs existed
    end
end
updateData();
if ~opt.useGui
    R = makeResult();
    return;
end

% ---------------------------------------------------------------- window
fig = figure('Name', ['Gait phase: ' name], 'NumberTitle', 'off', ...
    'Units', 'normalized', 'OuterPosition', [0.02 0.05 0.96 0.9], 'MenuBar', 'none', ...
    'ToolBar', 'figure', 'CloseRequestFcn', @onClose, 'WindowKeyPressFcn', @onKey, ...
    'WindowScrollWheelFcn', @onScroll);
pnl = uipanel(fig, 'Units', 'normalized', 'Position', [0.005 0.01 0.175 0.98], 'BorderType', 'none');
ctl('text', 'Reference angular velocity', [0.04 0.955 0.92 0.03], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
popAxis = ctl('popupmenu', axisNames, [0.04 0.915 0.92 0.04], 'Value', axisChoice, 'Callback', @onAxis);
cbInv   = ctl('checkbox', 'Invert signal', [0.04 0.875 0.92 0.035], 'Value', invert, 'Callback', @onInvert);
ctl('pushbutton', 'Re-run automatic detection', [0.04 0.83 0.92 0.04], 'Callback', @onRedetect);
ctl('pushbutton', 'Re-detect toe-offs only', [0.04 0.785 0.92 0.04], 'Callback', @onRedetectTo);
bg = uibuttongroup(pnl, 'Units', 'normalized', 'Position', [0.04 0.555 0.92 0.22], ...
    'Title', 'Click on the signal to', 'FontSize', 10, ...
    'SelectionChangedFcn', @onMode);
modes  = {'nav', 'addHs', 'delHs', 'addTo', 'delTo'};
labels = {'Navigate only (n)', 'Add heel strike (a)', 'Delete heel strike (d)', ...
    'Add toe-off (t)', 'Remove toe-off (r)'};
for im = 1:numel(modes)
    rb.(modes{im}) = uicontrol(bg, 'Style', 'radiobutton', 'String', labels{im}, 'Units', 'normalized', ...
        'Position', [0.05 0.81 - 0.195 * (im - 1) 0.9 0.17], 'FontSize', 10, 'UserData', modes{im});
end
ctl('text', 'Window', [0.04 0.51 0.3 0.03], 'HorizontalAlignment', 'left');
iw = find(winWidths == winW, 1); if isempty(iw), iw = 2; winW = winWidths(iw); end
popWin = ctl('popupmenu', winLabels, [0.36 0.515 0.6 0.035], 'Value', iw, 'Callback', @onWinWidth);
ctl('pushbutton', '< Back', [0.04 0.465 0.45 0.04], 'Callback', @(~, ~) pan(-0.9));
ctl('pushbutton', 'Forward >', [0.51 0.465 0.45 0.04], 'Callback', @(~, ~) pan(0.9));
ctl('pushbutton', '< Suspicious', [0.04 0.42 0.45 0.04], 'Callback', @(~, ~) gotoSusp(-1));
ctl('pushbutton', 'Suspicious > (s)', [0.51 0.42 0.45 0.04], 'Callback', @(~, ~) gotoSusp(1));
ctl('pushbutton', 'Undo (u)', [0.04 0.375 0.92 0.04], 'Callback', @(~, ~) undo());
txtStats = ctl('text', '', [0.04 0.135 0.92 0.235], 'HorizontalAlignment', 'left', 'Max', 2);
ctl('pushbutton', 'Save and close', [0.04 0.075 0.92 0.05], 'FontWeight', 'bold', 'Callback', @onSave);
ctl('pushbutton', 'Cancel', [0.04 0.02 0.92 0.045], 'Callback', @(~, ~) onClose());

ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.225 0.43 0.76 0.53], ...
    'ButtonDownFcn', @onSigClick);
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
hBad   = patch(ax1, NaN, NaN, [1 0.3 0.3], 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'PickableParts', 'none');
hBadTo = patch(ax1, NaN, NaN, cTO, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'PickableParts', 'none');
hSig   = plot(ax1, t, w, 'Color', [0.1 0.35 0.75], 'LineWidth', 1, 'PickableParts', 'none');
hLines = plot(ax1, NaN, NaN, ':', 'Color', cHS, 'PickableParts', 'none');
hToLines = plot(ax1, NaN, NaN, '--', 'Color', cTO, 'PickableParts', 'none');
hHS    = plot(ax1, NaN, NaN, 'o', 'MarkerFaceColor', cHS, 'MarkerEdgeColor', 'k', ...
    'MarkerSize', 7, 'PickableParts', 'none');
hTO    = plot(ax1, NaN, NaN, 's', 'MarkerFaceColor', cTO, 'MarkerEdgeColor', 'k', ...
    'MarkerSize', 7, 'PickableParts', 'none');
ylabel(ax1, 'reference [rad/s]');
ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.225 0.255 0.76 0.13]);
hold(ax2, 'on');
hPh   = plot(ax2, t, ph, 'Color', [0.85 0.33 0.1], 'LineWidth', 1, 'PickableParts', 'none');
hPhTo = plot(ax2, NaN, NaN, 's', 'MarkerFaceColor', cTO, 'MarkerEdgeColor', 'k', ...
    'MarkerSize', 5, 'PickableParts', 'none');
grid(ax2, 'on'); box(ax2, 'on'); ylim(ax2, [0 100]); ylabel(ax2, 'phase [%]');
ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.225 0.055 0.76 0.14], ...
    'ButtonDownFcn', @onOverviewClick);
hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
hWin  = patch(ax3, NaN, NaN, [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', [0.5 0.5 0.5], 'PickableParts', 'none');
hT    = plot(ax3, NaN, NaN, '.-', 'Color', [0.5 0.5 0.5], 'PickableParts', 'none');
hTbad = plot(ax3, NaN, NaN, 'o', 'MarkerEdgeColor', cHS, 'MarkerFaceColor', [1 0.6 0.6], ...
    'PickableParts', 'none');
hTto  = plot(ax3, NaN, NaN, 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', cTO, ...
    'PickableParts', 'none');
xlim(ax3, [tLo tHi]);
xlabel(ax3, 't\_sim [s]   (click here to jump)'); ylabel(ax3, 'stride [s]');
linkaxes([ax1 ax2], 'x');
if exist('disableDefaultInteractivity', 'file') || exist('disableDefaultInteractivity', 'builtin')
    disableDefaultInteractivity(ax1); disableDefaultInteractivity(ax2); disableDefaultInteractivity(ax3);
end
setappdata(fig, 'gait_api', struct('add', @(tc) addAt(tc, 'hs'), 'del', @(tc) delAt(tc, 'hs'), ...
    'addTo', @(tc) addAt(tc, 'to'), 'delTo', @(tc) delAt(tc, 'to'), 'delAny', @delNearest, ...
    'undo', @undo, 'redetectTo', @() redetectTo(), ...
    'setAxis', @(k) applyReference(k, invertSuggestion(k)), 'setInvert', @(b) applyReference(axisChoice, b), ...
    'gotoSusp', @gotoSusp, 'save', @() onSave(), 'state', @getState));

redrawSignal();
redrawEvents();
setWindow();
uiwait(fig);
if ishandle(fig), delete(fig); end

% =========================================================== state updates
    function computeSignal()
        [w, axVec, sigName] = gait_reference_signal(gyro, axisChoice, invert, opt.lpHz, fs);
        yl = [vn_prctile(w, 0.02) vn_prctile(w, 99.98)];
        yl = yl + [-0.08 0.08] * diff(yl);
    end

    function runAuto()
        [hsAuto, ms] = gait_hs_detect(t, w, opt);
        hs = hsAuto;
        runAutoTo();
        undoStack = {};
    end

    function runAutoTo()
        toAuto = gait_to_detect(t, w, hs, opt);
        to = toAuto;
    end

    function updateData()
        hsT = gait_refine_min(t, w, hs);
        toT = gait_refine_min(t, w, to);
        [ph, ~, okS, T] = gait_phase_compute(hsT, t, opt.strideRange);
        % toe-offs per HS interval: a valid stride should hold exactly one
        nTo = zeros(numel(T), 1);
        if numel(hsT) >= 2 && ~isempty(toT)
            k = discretize(toT, hsT);
            k = k(~isnan(k) & k <= numel(T));
            nTo = accumarray(k(:), 1, [numel(T) 1]);
        end
        badHs = find(~okS);
        badTo = find(okS & nTo ~= 1);
        susp  = sort([badHs; badTo]);
    end

    function e = isEdited()
        e = ~isequal(hs(:), hsAuto(:)) || ~isequal(to(:), toAuto(:));
    end

    function inv = invertSuggestion(k)
        w0  = gait_reference_signal(gyro, k, false, opt.lpHz, fs);
        inv = gait_auto_sign(w0, fs) < 0;
    end

    function applyReference(k, inv)
        axisChoice = k;
        invert = logical(inv);
        suggInv = invertSuggestion(k);
        computeSignal();
        runAuto();
        lastAction = sprintf('Reference %s: automatic detection re-run.', sigName);
        if ishandle(fig)
            popAxis.Value = axisChoice;
            cbInv.Value = invert;
            redrawSignal();
        end
        refresh();
    end

    function redetectTo()
        pushUndo();
        runAutoTo();
        lastAction = sprintf('Toe-offs re-detected from the current heel strikes (%d).', numel(to));
        refresh();
    end

    function refresh()
        updateData();
        if opt.useGui && ishandle(fig), redrawEvents(); end
    end

    function pushUndo()
        undoStack{end + 1} = struct('hs', hs, 'to', to, 'toAuto', toAuto);
    end

    function h = ctl(style, str, pos, varargin)
        h = uicontrol(pnl, 'Style', style, 'String', str, 'Units', 'normalized', ...
            'Position', pos, 'FontSize', 10, varargin{:});
    end

% ================================================================ editing
    function addAt(tc, kind)
        [~, i0] = min(abs(t - tc));
        m = round(opt.snapWin * fs);
        r = max(1, i0 - m):min(n, i0 + m);
        [~, j] = min(w(r));
        i = r(j);
        if strcmp(kind, 'hs'), ev = hs; what = 'heel strike'; else, ev = to; what = 'toe-off'; end
        if ~isempty(ev) && min(abs(t(ev) - t(i))) < opt.minHsSep
            lastAction = sprintf('Not added: a %s already exists within %.2f s of %.3f s.', what, opt.minHsSep, t(i));
        else
            pushUndo();
            if strcmp(kind, 'hs'), hs = sort([hs; i]); else, to = sort([to; i]); end
            lastAction = sprintf('Added %s at %.3f s.', what, t(i));
        end
        refresh();
    end

    function delAt(tc, kind)
        if strcmp(kind, 'hs'), ev = hs; what = 'heel strike'; else, ev = to; what = 'toe-off'; end
        if isempty(ev)
            lastAction = sprintf('No %ss to delete.', what);
        else
            [d, j] = min(abs(t(ev) - tc));
            if d > opt.delTol
                lastAction = sprintf('Not deleted: no %s within %.2f s of the click.', what, opt.delTol);
            else
                pushUndo();
                lastAction = sprintf('Deleted %s at %.3f s.', what, t(ev(j)));
                if strcmp(kind, 'hs'), hs(j) = []; else, to(j) = []; end
            end
        end
        refresh();
    end

    function delNearest(tc)
        dh = Inf; dt = Inf;
        if ~isempty(hs), dh = min(abs(t(hs) - tc)); end
        if ~isempty(to), dt = min(abs(t(to) - tc)); end
        if dt < dh, delAt(tc, 'to'); else, delAt(tc, 'hs'); end
    end

    function undo()
        if isempty(undoStack)
            lastAction = 'Nothing to undo.';
        else
            u = undoStack{end};
            hs = u.hs; to = u.to; toAuto = u.toAuto;
            undoStack(end) = [];
            lastAction = 'Undone.';
        end
        refresh();
    end

    function S = getState()
        S = struct('hs', hs, 'hsAuto', hsAuto, 'to', to, 'toAuto', toAuto, ...
            'axisChoice', axisChoice, 'invert', invert, 'susp', susp, 'badHs', badHs, ...
            'badTo', badTo, 'winC', winC, 'winW', winW, 'lastAction', lastAction);
    end

    function R1 = makeResult()
        R1 = struct('cancelled', false, 'hsIdx', hs, 'hsTime', hsT, 'hsAutoIdx', hsAuto, ...
            'hsAutoTime', gait_refine_min(t, w, hsAuto), 'msIdx', ms, ...
            'toIdx', to, 'toTime', toT, 'toAutoIdx', toAuto, ...
            'toAutoTime', gait_refine_min(t, w, toAuto), 'axisChoice', axisChoice, ...
            'axisName', axisNames{axisChoice}, 'signalName', sigName, 'axisVec', axVec, ...
            'invert', invert, 'nAdded', numel(setdiff(hs, hsAuto)), ...
            'nDeleted', numel(setdiff(hsAuto, hs)), 'nToAdded', numel(setdiff(to, toAuto)), ...
            'nToDeleted', numel(setdiff(toAuto, to)), 'w', w);
    end

% ================================================================ drawing
    function redrawSignal()
        set(hSig, 'YData', w);
        ylim(ax1, yl);
        ylabel(ax1, sprintf('%s [rad/s], %g Hz low-pass', sigName, opt.lpHz));
    end

    function [X, Y] = vlines(tv)
        if isempty(tv), X = NaN; Y = NaN; return; end
        X = [tv(:) tv(:) nan(numel(tv), 1)].';
        Y = repmat([yl(1); yl(2); NaN], 1, numel(tv));
        X = X(:); Y = Y(:);
    end

    function shade(h, ks)
        if isempty(ks)
            set(h, 'XData', NaN, 'YData', NaN);
        else
            a = hsT(ks).'; b = hsT(ks + 1).';
            set(h, 'XData', [a; b; b; a], 'YData', repmat([yl(1); yl(1); yl(2); yl(2)], 1, numel(a)));
        end
    end

    function redrawEvents()
        set(hHS, 'XData', hsT, 'YData', w(hs));
        set(hTO, 'XData', toT, 'YData', w(to));
        [X, Y] = vlines(hsT); set(hLines, 'XData', X, 'YData', Y);
        [X, Y] = vlines(toT); set(hToLines, 'XData', X, 'YData', Y);
        shade(hBad, badHs);
        shade(hBadTo, badTo);
        set(hPh, 'YData', ph);
        set(hPhTo, 'XData', toT, 'YData', ph(to));
        stance = NaN;
        if numel(hsT) >= 2
            Tmed  = median(T);
            Tclip = 2.2 * Tmed;
            set(hT, 'XData', hsT(1:end - 1), 'YData', min(T, Tclip));
            set(hTbad, 'XData', hsT(badHs), 'YData', min(T(badHs), Tclip));
            set(hTto, 'XData', hsT(badTo), 'YData', min(T(badTo), Tclip));
            ylim(ax3, [0 Tclip * 1.05]);
            pTo = ph(to);
            stance = median(pTo(isfinite(pTo)));
        else
            Tmed = NaN;
            set(hT, 'XData', NaN, 'YData', NaN);
            set(hTbad, 'XData', NaN, 'YData', NaN);
            set(hTto, 'XData', NaN, 'YData', NaN);
        end
        modeTxt = struct('nav', 'NAVIGATE (a/d = heel strike, t/r = toe-off)', ...
            'addHs', 'ADD HEEL STRIKE: click near a trough', 'delHs', 'DELETE HEEL STRIKE: click near one', ...
            'addTo', 'ADD TOE-OFF: click near a trough', 'delTo', 'REMOVE TOE-OFF: click near one');
        title(ax1, sprintf('%s  |  %s', strrep(name, '_', '\_'), modeTxt.(clickMode)));
        sugg = 'off';
        if suggInv, sugg = 'on'; end
        set(txtStats, 'String', { ...
            sprintf('Heel strikes: %d (auto %d, +%d, -%d)', numel(hs), numel(hsAuto), ...
                numel(setdiff(hs, hsAuto)), numel(setdiff(hsAuto, hs))), ...
            sprintf('Toe-offs: %d (auto %d, +%d, -%d)', numel(to), numel(toAuto), ...
                numel(setdiff(to, toAuto)), numel(setdiff(toAuto, to))), ...
            sprintf('Strides: %d valid, %d bad length', sum(okS), numel(badHs)), ...
            sprintf('Valid strides without 1 toe-off: %d', numel(badTo)), ...
            sprintf('Median stride %.3f s', Tmed), ...
            sprintf('Median toe-off at %.1f %% of stride', stance), ...
            sprintf('Auto sign suggests Invert %s', sugg), ...
            '', lastAction});
        updateWinPatch();
    end

    function setWindow()
        if isinf(winW)
            xl = [tLo tHi];
        else
            winC = min(max(winC, tLo + winW / 2), tHi - winW / 2);
            xl = winC + [-0.5 0.5] * winW;
        end
        set(ax1, 'XLim', xl);
        updateWinPatch();
    end

    function updateWinPatch()
        xl = get(ax1, 'XLim');
        yt = get(ax3, 'YLim');
        set(hWin, 'XData', [xl(1) xl(2) xl(2) xl(1)], 'YData', [yt(1) yt(1) yt(2) yt(2)]);
    end

% ============================================================== callbacks
    function onAxis(src, ~)
        k = src.Value;
        if k == axisChoice, return; end
        if isEdited() && ~confirm('heel-strike and toe-off')
            src.Value = axisChoice;
            return;
        end
        applyReference(k, invertSuggestion(k));
    end

    function onInvert(src, ~)
        if isEdited() && ~confirm('heel-strike and toe-off')
            src.Value = invert;
            return;
        end
        applyReference(axisChoice, logical(src.Value));
    end

    function onRedetect(~, ~)
        if isEdited() && ~confirm('heel-strike and toe-off'), return; end
        runAuto();
        lastAction = 'Automatic detection re-run.';
        refresh();
    end

    function onRedetectTo(~, ~)
        if ~isequal(to(:), toAuto(:)) && ~confirm('toe-off'), return; end
        redetectTo();
    end

    function ok = confirm(what)
        a = questdlg(sprintf(['This re-runs the automatic detection and discards your manual ' ...
            '%s edits. Continue?'], what), 'Gait phase', 'Continue', 'Cancel', 'Cancel');
        ok = strcmp(a, 'Continue');
    end

    function onMode(~, ev)
        clickMode = ev.NewValue.UserData;
        redrawEvents();
    end

    function setMode(m)
        clickMode = m;
        bg.SelectedObject = rb.(m);
        redrawEvents();
    end

    function onSigClick(~, ~)
        cp = get(ax1, 'CurrentPoint');
        tc = cp(1, 1);
        if strcmp(get(fig, 'SelectionType'), 'alt')
            delNearest(tc);
            return;
        end
        switch clickMode
            case 'addHs', addAt(tc, 'hs');
            case 'delHs', delAt(tc, 'hs');
            case 'addTo', addAt(tc, 'to');
            case 'delTo', delAt(tc, 'to');
            otherwise
                lastAction = 'Navigate mode: choose an Add or Delete mode to edit (right-click deletes).';
                redrawEvents();
        end
    end

    function onOverviewClick(~, ~)
        cp = get(ax3, 'CurrentPoint');
        winC = cp(1, 1);
        if isinf(winW), winW = 10; popWin.Value = 2; end
        setWindow();
    end

    function onWinWidth(src, ~)
        winW = winWidths(src.Value);
        setWindow();
    end

    function pan(f)
        if isinf(winW), return; end
        winC = winC + f * winW;
        setWindow();
    end

    function gotoSusp(dirn)
        if isempty(susp)
            lastAction = 'No suspicious strides.';
        else
            mids = (hsT(susp) + hsT(susp + 1)) / 2;
            if dirn > 0
                j = find(mids > winC + 1e-3, 1, 'first');
                if isempty(j), j = 1; end
            else
                j = find(mids < winC - 1e-3, 1, 'last');
                if isempty(j), j = numel(mids); end
            end
            if isinf(winW), winW = 10; if ~isempty(popWin), popWin.Value = 2; end; end
            winC = mids(j);
            k = susp(j);
            if ~okS(k)
                why = sprintf('%.3f s long (median %.3f s)', T(k), median(T));
            else
                why = sprintf('%d toe-offs', nTo(k));
            end
            lastAction = sprintf('Suspicious stride %d of %d at %.2f s: %s.', j, numel(susp), hsT(k), why);
        end
        if opt.useGui && ishandle(fig), setWindow(); redrawEvents(); end
    end

    function onKey(~, ev)
        switch ev.Key
            case 'leftarrow',  pan(-0.5);
            case 'rightarrow', pan(0.5);
            case 'a', setMode('addHs');
            case 'd', setMode('delHs');
            case 't', setMode('addTo');
            case 'r', setMode('delTo');
            case 'n', setMode('nav');
            case 'u', undo();
            case 'z'
                if any(strcmp(ev.Modifier, 'control')) || any(strcmp(ev.Modifier, 'command')), undo(); end
            case 's', gotoSusp(1);
        end
    end

    function onScroll(~, ev)
        pan(0.2 * ev.VerticalScrollCount);
    end

    function onSave(~, ~)
        R = makeResult();
        uiresume(fig);
    end

    function onClose(~, ~)
        if isEdited()
            a = questdlg('Close without saving? Your heel-strike and toe-off edits will be lost.', ...
                'Gait phase', 'Discard', 'Keep editing', 'Keep editing');
            if ~strcmp(a, 'Discard'), return; end
        end
        R = struct('cancelled', true);
        uiresume(fig);
    end
end
