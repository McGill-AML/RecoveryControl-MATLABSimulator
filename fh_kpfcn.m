function [] = fh_kpfcn(H,E)          
% Figure keypressfcn
S = guidata(H);
P = get(S.fh,'position');
% set(S.tx,'string',E.Key)
switch E.Key
    case 'rightarrow'
        set(S.fh,'pos',P+[5 0 0 0])
    case 'leftarrow'
        set(S.fh,'pos',P+[-5 0 0 0])
    case 'uparrow'
        set(S.fh,'pos',P+[0 5 0 0])
    case 'downarrow'
        set(S.fh,'pos',P+[0 -5 0 0])
    otherwise  
end