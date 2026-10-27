function R = gait_phase_editor(t, gyro, S0, opt, name)
%GAIT_PHASE_EDITOR  Review and edit heel strikes for the continuous gait phase.
%   R = gait_phase_editor(t, gyro, S0, opt, name)
%     t     [n x 1] IMU time (t_sim) [s]       gyro  [n x 3] shoe angular rate [rad/s]
%     S0    start state: .axisChoice (1/2/3 = gyro x/y/z, 4 = principal axis),
%           .invert (true/false), .hsIdx (sample indices; [] = automatic detection),
%           .hsAutoIdx (optional, automatic set of a resumed file)
%     opt   options of gait_phase_run (lpHz, strideRange, snapWin, minHsSep,
%           delTol, winWidth, useGui and the gait_hs_detect fields)
%     name  shown in the window title
%   With opt.useGui = false the automatic detection is returned directly.
%   Otherwise the window blocks until "Save and close" (R.cancelled = false)
%   or "Cancel" / closing the window (R.cancelled = true).
%
%   Window
%     left panel   reference signal (gyro x/y/z or principal axis), Invert,
%                  re-run automatic detection, click mode, window width,
%                  navigation, undo, counts, Save and close / Cancel
%     top axes     reference signal with the heel strikes (red); invalid
%                  strides shaded red.  Add mode: a click adds a heel strike at
%                  the lowest point within opt.snapWin s.  Delete mode: a click
%                  removes the nearest heel strike (within opt.delTol s).
%                  Right-click (ctrl-click) deletes in any mode.
%     middle axes  resulting gait phase [%]
%     bottom axes  stride time over the whole trial (red = outside
%                  opt.strideRange x median); click to jump there
%   Keys: left/right arrow pan, mouse wheel pans, a/d/n = add/delete/navigate,
%   u = undo, s = next suspicious stride.

t    = t(:);
gyro = double(gyro);
n    = numel(t);
fs   = 1 / median(diff(t));
axisNames  = {'Gyro x', 'Gyro y', 'Gyro z', 'Principal axis (PCA)'};
winWidths  = [5 10 20 60 Inf];
winLabels  = {'5 s', '10 s', '20 s', '60 s', 'Whole trial'};

axisChoice = S0.axisChoice;
invert     = logical(S0.invert);
w = []; axVec = []; sigName = '';
hs = zeros(0, 1); hsAuto = zeros(0, 1); ms = zeros(0, 1);
hsT = zeros(0, 1); T = zeros(0, 1); okS = false(0, 1); susp = zeros(0, 1); ph = [];
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
bg = uibuttongroup(pnl, 'Units', 'normalized', 'Position', [0.04 0.66 0.92 0.155], ...
    'Title', 'Click on the signal to', 'FontSize', 10, ...
    'SelectionChangedFcn', @onMode);
rb.nav = uicontrol(bg, 'Style', 'radiobutton', 'String', 'Navigate only (n)', 'Units', 'normalized', ...
    'Position', [0.05 0.68 0.9 0.28], 'FontSize', 10, 'UserData', 'nav');
rb.add = uicontrol(bg, 'Style', 'radiobutton', 'String', 'Add heel strike (a)', 'Units', 'normalized', ...
    'Position', [0.05 0.37 0.9 0.28], 'FontSize', 10, 'UserData', 'add');
rb.del = uicontrol(bg, 'Style', 'radiobutton', 'String', 'Delete heel strike (d)', 'Units', 'normalized', ...
    'Position', [0.05 0.06 0.9 0.28], 'FontSize', 10, 'UserData', 'del');
ctl('text', 'Window', [0.04 0.615 0.3 0.03], 'HorizontalAlignment', 'left');
iw = find(winWidths == winW, 1); if isempty(iw), iw = 2; winW = winWidths(iw); end
popWin = ctl('popupmenu', winLabels, [0.36 0.62 0.6 0.035], 'Value', iw, 'Callback', @onWinWidth);
ctl('pushbutton', '< Back', [0.04 0.57 0.45 0.04], 'Callback', @(~, ~) pan(-0.9));
ctl('pushbutton', 'Forward >', [0.51 0.57 0.45 0.04], 'Callback', @(~, ~) pan(0.9));
ctl('pushbutton', '< Suspicious', [0.04 0.525 0.45 0.04], 'Callback', @(~, ~) gotoSusp(-1));
ctl('pushbutton', 'Suspicious > (s)', [0.51 0.525 0.45 0.04], 'Callback', @(~, ~) gotoSusp(1));
ctl('pushbutton', 'Undo (u)', [0.04 0.48 0.92 0.04], 'Callback', @(~, ~) undo());
txtStats = ctl('text', '', [0.04 0.17 0.92 0.30], 'HorizontalAlignment', 'left', 'Max', 2);
ctl('pushbutton', 'Save and close', [0.04 0.075 0.92 0.05], 'FontWeight', 'bold', 'Callback', @onSave);
ctl('pushbutton', 'Cancel', [0.04 0.02 0.92 0.045], 'Callback', @(~, ~) onClose());

ax1 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.225 0.43 0.76 0.53], ...
    'ButtonDownFcn', @onSigClick);
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
hBad   = patch(ax1, NaN, NaN, [1 0.3 0.3], 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'PickableParts', 'none');
hSig   = plot(ax1, t, w, 'Color', [0.1 0.35 0.75], 'LineWidth', 1, 'PickableParts', 'none');
hLines = plot(ax1, NaN, NaN, ':', 'Color', [0.85 0.1 0.1], 'PickableParts', 'none');
hHS    = plot(ax1, NaN, NaN, 'o', 'MarkerFaceColor', [0.85 0.1 0.1], 'MarkerEdgeColor', 'k', ...
    'MarkerSize', 7, 'PickableParts', 'none');
ylabel(ax1, 'reference [rad/s]');
ax2 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.225 0.255 0.76 0.13]);
hPh = plot(ax2, t, ph, 'Color', [0.85 0.33 0.1], 'LineWidth', 1, 'PickableParts', 'none');
grid(ax2, 'on'); ylim(ax2, [0 100]); ylabel(ax2, 'phase [%]');
ax3 = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.225 0.055 0.76 0.14], ...
    'ButtonDownFcn', @onOverviewClick);
hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
hWin  = patch(ax3, NaN, NaN, [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', [0.5 0.5 0.5], 'PickableParts', 'none');
hT    = plot(ax3, NaN, NaN, '.-', 'Color', [0.5 0.5 0.5], 'PickableParts', 'none');
hTbad = plot(ax3, NaN, NaN, 'o', 'MarkerEdgeColor', [0.85 0.1 0.1], 'MarkerFaceColor', [1 0.6 0.6], ...
    'PickableParts', 'none');
xlim(ax3, [tLo tHi]);
xlabel(ax3, 't\_sim [s]   (click here to jump)'); ylabel(ax3, 'stride [s]');
linkaxes([ax1 ax2], 'x');
if exist('disableDefaultInteractivity', 'file') || exist('disableDefaultInteractivity', 'builtin')
    disableDefaultInteractivity(ax1); disableDefaultInteractivity(ax2); disableDefaultInteractivity(ax3);
end
setappdata(fig, 'gait_api', struct('add', @addAt, 'del', @delAt, 'undo', @undo, ...
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
        undoStack = {};
    end

    function updateData()
        hsT = gait_refine_min(t, w, hs);
        [ph, ~, okS, T] = gait_phase_compute(hsT, t, opt.strideRange);
        susp = find(~okS);
    end

    function e = isEdited()
        e = ~isequal(hs(:), hsAuto(:));
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

    function refresh()
        updateData();
        if opt.useGui && ishandle(fig), redrawEvents(); end
    end

    function pushUndo()
        undoStack{end + 1} = hs;
    end

    function h = ctl(style, str, pos, varargin)
        h = uicontrol(pnl, 'Style', style, 'String', str, 'Units', 'normalized', ...
            'Position', pos, 'FontSize', 10, varargin{:});
    end

% ================================================================ editing
    function addAt(tc)
        [~, i0] = min(abs(t - tc));
        m = round(opt.snapWin * fs);
        r = max(1, i0 - m):min(n, i0 + m);
        [~, j] = min(w(r));
        i = r(j);
        if ~isempty(hs) && min(abs(t(hs) - t(i))) < opt.minHsSep
            lastAction = sprintf('Not added: a heel strike already exists within %.2f s of %.3f s.', opt.minHsSep, t(i));
        else
            pushUndo();
            hs = sort([hs; i]);
            lastAction = sprintf('Added heel strike at %.3f s.', t(i));
        end
        refresh();
    end

    function delAt(tc)
        if isempty(hs)
            lastAction = 'No heel strikes to delete.';
        else
            [d, j] = min(abs(t(hs) - tc));
            if d > opt.delTol
                lastAction = sprintf('Not deleted: no heel strike within %.2f s of the click.', opt.delTol);
            else
                pushUndo();
                lastAction = sprintf('Deleted heel strike at %.3f s.', t(hs(j)));
                hs(j) = [];
            end
        end
        refresh();
    end

    function undo()
        if isempty(undoStack)
            lastAction = 'Nothing to undo.';
        else
            hs = undoStack{end};
            undoStack(end) = [];
            lastAction = 'Undone.';
        end
        refresh();
    end

    function S = getState()
        S = struct('hs', hs, 'hsAuto', hsAuto, 'axisChoice', axisChoice, 'invert', invert, ...
            'susp', susp, 'winC', winC, 'winW', winW, 'lastAction', lastAction);
    end

    function R1 = makeResult()
        R1 = struct('cancelled', false, 'hsIdx', hs, 'hsTime', hsT, 'hsAutoIdx', hsAuto, ...
            'hsAutoTime', gait_refine_min(t, w, hsAuto), 'msIdx', ms, 'axisChoice', axisChoice, ...
            'axisName', axisNames{axisChoice}, 'signalName', sigName, 'axisVec', axVec, ...
            'invert', invert, 'nAdded', numel(setdiff(hs, hsAuto)), ...
            'nDeleted', numel(setdiff(hsAuto, hs)), 'w', w);
    end

% ================================================================ drawing
    function redrawSignal()
        set(hSig, 'YData', w);
        ylim(ax1, yl);
        ylabel(ax1, sprintf('%s [rad/s], %g Hz low-pass', sigName, opt.lpHz));
    end

    function redrawEvents()
        set(hHS, 'XData', hsT, 'YData', w(hs));
        if isempty(hsT)
            set(hLines, 'XData', NaN, 'YData', NaN);
        else
            X = [hsT hsT nan(size(hsT))].';
            Y = repmat([yl(1); yl(2); NaN], 1, numel(hsT));
            set(hLines, 'XData', X(:), 'YData', Y(:));
        end
        if isempty(susp)
            set(hBad, 'XData', NaN, 'YData', NaN);
        else
            a = hsT(susp).'; b = hsT(susp + 1).';
            set(hBad, 'XData', [a; b; b; a], 'YData', repmat([yl(1); yl(1); yl(2); yl(2)], 1, numel(a)));
        end
        set(hPh, 'YData', ph);
        if numel(hsT) >= 2
            Tmed  = median(T);
            Tclip = 2.2 * Tmed;
            set(hT, 'XData', hsT(1:end - 1), 'YData', min(T, Tclip));
            set(hTbad, 'XData', hsT(susp), 'YData', min(T(susp), Tclip));
            ylim(ax3, [0 Tclip * 1.05]);
        else
            Tmed = NaN;
            set(hT, 'XData', NaN, 'YData', NaN);
            set(hTbad, 'XData', NaN, 'YData', NaN);
        end
        modeTxt = struct('nav', 'NAVIGATE (a = add, d = delete)', 'add', 'ADD: click near a trough', ...
            'del', 'DELETE: click near a heel strike');
        title(ax1, sprintf('%s  |  %s', strrep(name, '_', '\_'), modeTxt.(clickMode)));
        sugg = 'off';
        if suggInv, sugg = 'on'; end
        set(txtStats, 'String', { ...
            sprintf('Heel strikes: %d', numel(hs)), ...
            sprintf('  automatic %d, added %d, deleted %d', numel(hsAuto), ...
                numel(setdiff(hs, hsAuto)), numel(setdiff(hsAuto, hs))), ...
            sprintf('Strides: %d valid, %d suspicious', sum(okS), numel(susp)), ...
            sprintf('Median stride: %.3f s', Tmed), ...
            sprintf('Valid range: %.2f-%.2f x median', opt.strideRange(1), opt.strideRange(2)), ...
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
        if isEdited() && ~confirm()
            src.Value = axisChoice;
            return;
        end
        applyReference(k, invertSuggestion(k));
    end

    function onInvert(src, ~)
        if isEdited() && ~confirm()
            src.Value = invert;
            return;
        end
        applyReference(axisChoice, logical(src.Value));
    end

    function onRedetect(~, ~)
        if isEdited() && ~confirm(), return; end
        runAuto();
        lastAction = 'Automatic detection re-run.';
        refresh();
    end

    function ok = confirm()
        a = questdlg(['This re-runs the automatic detection and discards your manual ' ...
            'heel-strike edits. Continue?'], 'Gait phase', 'Continue', 'Cancel', 'Cancel');
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
            delAt(tc);
            return;
        end
        switch clickMode
            case 'add', addAt(tc);
            case 'del', delAt(tc);
            otherwise
                lastAction = 'Navigate mode: choose Add or Delete to edit (right-click deletes).';
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
            lastAction = sprintf('Suspicious stride %d of %d at %.2f s: %.3f s long (median %.3f s).', ...
                j, numel(susp), hsT(susp(j)), T(susp(j)), median(T));
        end
        if opt.useGui && ishandle(fig), setWindow(); redrawEvents(); end
    end

    function onKey(~, ev)
        switch ev.Key
            case 'leftarrow',  pan(-0.5);
            case 'rightarrow', pan(0.5);
            case 'a', setMode('add');
            case 'd', setMode('del');
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
            a = questdlg('Close without saving? Your heel-strike edits will be lost.', ...
                'Gait phase', 'Discard', 'Keep editing', 'Keep editing');
            if ~strcmp(a, 'Discard'), return; end
        end
        R = struct('cancelled', true);
        uiresume(fig);
    end
end
