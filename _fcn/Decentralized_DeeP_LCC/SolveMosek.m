function [res1,mosektime] = SolveMosek(At,b,c,K)
    %Input: sedumi format
    prob1 = SedumiToMosek_Latest(At,b,c,K);
    tic 
    [~, res1] = mosekopt('minimize echo(0)', prob1);
%     [~, res1] = mosekopt('minimize', prob1);
    mosektime = toc; 
end