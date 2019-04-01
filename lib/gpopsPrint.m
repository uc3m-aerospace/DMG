function gpopsPrint(setup);
%------------------------------------------------------------------%
% Prints the information about a multiple-phase optimal control    %
% problem to a filename contained in the string SETUP.NAME         %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin % 
%------------------------------------------------------------------%

sizes  = setup.sizes;
limits = setup.limits;
numphases = setup.numphases;
filename = strcat([setup.name,'.txt']);
fid = fopen(filename,'w');
fprintf(fid,'\n');
ssfilename = strcat(['Summary of Problem Written to File: ',filename]);
strdashed = '';
for k=1:length(ssfilename)
    strdashed = strcat([strdashed,'-']);
end
disp(strdashed);
disp(ssfilename);
disp(strdashed);
% Print the license information to the file "name.txt" where "name"
% is the name of the problem being solved. 
fprintf(fid,'\n');
fprintf(fid,'%s\n','%------------------------------------------------------------------%');
fprintf(fid,'%s\n','%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%');
fprintf(fid,'%s\n','% THIS NOTICE MAY NOT BE DELETED UNDER CIRCUMSTANCES DURING ANY    %');
fprintf(fid,'%s\n','% EXECUTION OF THE GPOPS SOFTWARE OR TRANSMITTAL OF GPOPS OR ANY   %');
fprintf(fid,'%s\n','% DOCUMENTATION ASSOCIATED WITH GPOPS                              %');
fprintf(fid,'%s\n','%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%');
fprintf(fid,'%s\n','%------------------------------------------------------------------%');
fprintf(fid,'%s\n','% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %');
fprintf(fid,'%s\n','% Benson, Michael Patterson, and Christopher Darby.                %');
fprintf(fid,'%s\n','%------------------------------------------------------------------%');
fprintf(fid,'%s\n','% License for GPOPS Software (Based on the Simple Public License)  %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% Preamble                                                         %');
fprintf(fid,'%s\n','% This GPOPS License is based on the Simple Public License.  In    %');
fprintf(fid,'%s\n','% the same spirit as the Simple Public License, the language       %');
fprintf(fid,'%s\n','% implementation the GPOPS License is similar to that of GPL 2.0.  %');
fprintf(fid,'%s\n','% The words are different, but the goal is the same - to guarantee %');
fprintf(fid,'%s\n','% for all users the freedom to share and change software.  If      %');
fprintf(fid,'%s\n','% anyone wonders about the meaning of the GPOPS License, they      %');
fprintf(fid,'%s\n','% should interpret it as consistent with GPL 2.0.                  %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% GPOPS License                                                    %');
fprintf(fid,'%s\n','% The GPOPS License applies to the software"s source and object    %');
fprintf(fid,'%s\n','% code and comes with any rights that I have in it (other than     %');
fprintf(fid,'%s\n','% trademarks). You agree to the GPOPS License SimPL by copying,    %');
fprintf(fid,'%s\n','% distributing, or making a derivative work of the software.       %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% You get the royalty free right to:                               %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     * Use the software for any purpose;                          %');
fprintf(fid,'%s\n','%     * Make derivative works of it (this is called a              %');
fprintf(fid,'%s\n','%      "Derived Work");                                            %');
fprintf(fid,'%s\n','%     * Copy and distribute it and any Derived Work.               %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% If you distribute the software or a Derived Work, you must give  %');
fprintf(fid,'%s\n','% back to the community by:                                        %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     * Prominently noting the date of any changes you make;       %');
fprintf(fid,'%s\n','%     * Leaving other people"s copyright notices, warranty         %');
fprintf(fid,'%s\n','%       disclaimers, and license terms  in place;                  %');
fprintf(fid,'%s\n','%     * Providing the source code, build scripts, installation     %');
fprintf(fid,'%s\n','%       scripts, and interface definitions in a form that is easy  %');
fprintf(fid,'%s\n','%       to get and best to modify;                                 %');
fprintf(fid,'%s\n','%     * Licensing it to everyone under SimPL, or substantially     %');
fprintf(fid,'%s\n','%       similar terms (such as GPL 2.0), without adding further    %');
fprintf(fid,'%s\n','%       restrictions to the rights provided;                       %');
fprintf(fid,'%s\n','%     * Conspicuously announcing that it is available under        %');
fprintf(fid,'%s\n','%       that license.                                              %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% There are some things that you must shoulder:                    %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     * You get NO WARRANTIES. None of any kind;                   %');
fprintf(fid,'%s\n','%     * If the software damages you in any way, you may only       %');
fprintf(fid,'%s\n','%       recover direct damages up to the amount you paid for it    %');
fprintf(fid,'%s\n','%       (that is zero if you did not pay anything). You may not    %');
fprintf(fid,'%s\n','%       recover any other damages, including those called          %');
fprintf(fid,'%s\n','%       "consequential damages." (The state or country where you   %');
fprintf(fid,'%s\n','%       live may not allow you to limit your liability in this     %');
fprintf(fid,'%s\n','%       way, so this may not apply to you);                        %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% The GPOPS License continues perpetually, except that your        %');
fprintf(fid,'%s\n','% license rights end automatically if:                             %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     * You do not abide by the "give back to the community" terms %');
fprintf(fid,'%s\n','%       (your licensees get to keep their rights if they abide);   %');
fprintf(fid,'%s\n','%     * Anyone prevents you from distributing the software under   %');
fprintf(fid,'%s\n','%       the terms of the SimPL.                                    %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% THE GPOPS LICENSE IS NOT THE SIMPL, BUT IS BASED ON THE SIMPL    %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% In addition, to the license given above, the following is a      %');
fprintf(fid,'%s\n','% condition of using or modifying the GPOPS softare in any manner: %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%   Any results presented using GPOPS (i.e., conference papers,    %');
fprintf(fid,'%s\n','%   journal papers, oral presentations, or any other media) will   %');
fprintf(fid,'%s\n','%   include explicit citations to the fact that GPOPS was used     %');
fprintf(fid,'%s\n','%   to generate the results.  In particular, it is MANDATORY to    %');
fprintf(fid,'%s\n','%   cite the following references in any such media where          %');
fprintf(fid,'%s\n','%   results using GPOPS are presented:                             %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     [1] Benson, D. A., A Gauss Pseudospectral Transcription for  %');
fprintf(fid,'%s\n','%         Optimal Control, Ph.D. Thesis, Dept. of Aeronautics and  %');
fprintf(fid,'%s\n','%         Astronautics, MIT, November 2004.                        %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     [2] Huntington, G. T., Advancement and Analysis of a Gauss   %');
fprintf(fid,'%s\n','%         Pseudospectral Transcription for Optimal Control, Ph.D.  %');
fprintf(fid,'%s\n','%         Thesis, Dept. of Aeronautics and Astronautics, MIT,      %');
fprintf(fid,'%s\n','%         May 2007                                                 %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     [3] Benson, D. A., Huntington, G. T., Thorvaldsen, T. P.,    %');
fprintf(fid,'%s\n','%         and Rao, A. V., "Direct Trajectory Optimization and      %');
fprintf(fid,'%s\n','%         Costate Estimation via an Orthogonal Collocation Method, %');
fprintf(fid,'%s\n','%         Journal of Guidance, Control, and Dynamics, Vol. 29,     %');
fprintf(fid,'%s\n','%         No. 6, November-December 2006, pp. 1435-1440.            %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     [4] Huntington, G. T., Benson, D. A., and Rao, A. V.,        %');
fprintf(fid,'%s\n','%         Design of Optimal Tetrahedral Spacecraft Formations,     %');
fprintf(fid,'%s\n','%         Journal of the Astronautical Sciences, Vol. 55, No. 2,   %');
fprintf(fid,'%s\n','%         April-June 2007, pp. 141-169.                            %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     [5] Huntington, G. T., Benson, D. A., How, J. P.,            %');
fprintf(fid,'%s\n','%         Kanizay, N., Darby, C. L., and Rao, A. V.,               %');
fprintf(fid,'%s\n','%         "Computation of Boundary Controls Using a Gauss          %');
fprintf(fid,'%s\n','%         Pseudospectral Method," 2007 Astrodynamics Specialist    %');
fprintf(fid,'%s\n','%         Conference, Mackinac Island, Michigan, August 2007.      %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','%     [6] Huntington, G. T. and Rao, A. V., "Optimal               %');
fprintf(fid,'%s\n','%         Reconfiguration of Spacecraft Formations Using a Gauss   %');
fprintf(fid,'%s\n','%         Pseudospectral Method," Journal of Guidance, Control,    %');
fprintf(fid,'%s\n','%         & Dynamics, Vol. 31, No. 3, May-June 2008, pp. 689-698.  %');
fprintf(fid,'%s\n','%                                                                  %');
fprintf(fid,'%s\n','% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,  %');
fprintf(fid,'%s\n','% EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES  %');
fprintf(fid,'%s\n','% OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND         %');
fprintf(fid,'%s\n','% NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT      %');
fprintf(fid,'%s\n','% HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,     %');
fprintf(fid,'%s\n','% WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     %');
fprintf(fid,'%s\n','% FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR    %');
fprintf(fid,'%s\n','% OTHER DEALINGS IN THE SOFTWARE.                                  %');
fprintf(fid,'%s\n','%------------------------------------------------------------------%');
fprintf(fid,'%s\n','%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%');
fprintf(fid,'%s\n','% THIS NOTICE MAY NOT BE DELETED UNDER CIRCUMSTANCES DURING ANY    %');
fprintf(fid,'%s\n','% EXECUTION OF THE GPOPS SOFTWARE OR TRANSMITTAL OF GPOPS OR ANY   %');
fprintf(fid,'%s\n','% DOCUMENTATION ASSOCIATED WITH GPOPS                              %');
fprintf(fid,'%s\n','%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%');
fprintf(fid,'%s\n','%------------------------------------------------------------------%');
fprintf(fid,'%s\n','%------------------------------------------------------------------%');
fprintf(fid,'\n');
for i=1:numphases
    linestr =          '-----------------------\n';
    phasestr = strcat(['Information in Phase ',num2str(i),'\n']);
    fprintf(fid,'\n');
    fprintf(fid,linestr);
    fprintf(fid,phasestr);
    fprintf(fid,linestr);
    fprintf(fid,'\n');
    nstates = sizes(i,1);
    ncontrols = sizes(i,2);
    nparameters = sizes(i,3);
    npaths = sizes(i,4);
    nevents = sizes(i,5);
    if nstates>0,
        xlow1 = limits(i).state.min(:,1);
        xlow2 = limits(i).state.min(:,2);
        xlow3 = limits(i).state.min(:,3);
        xupp1 = limits(i).state.max(:,1);
        xupp2 = limits(i).state.max(:,2);
        xupp3 = limits(i).state.max(:,3);
        for j=1:nstates
            sxlow1 = num2str(xlow1(j));
            sxupp1 = num2str(xupp1(j));
            sxlow2 = num2str(xlow2(j));
            sxupp2 = num2str(xupp2(j));
            sxlow3 = num2str(xlow3(j));
            sxupp3 = num2str(xupp3(j));
            ee = ' <= ';
            sname = strcat(['State ',num2str(j)]);
            s1 = strcat(['\t Start of Phase:    \t',sxlow1,ee,sname,ee,sxupp1,'\n']);
            s2 = strcat(['\t During Phase:      \t',sxlow2,ee,sname,ee,sxupp2,'\n']);
            s3 = strcat(['\t Terminus of Phase: \t',sxlow3,ee,sname,ee,sxupp3,'\n']);
            fprintf(fid,strcat(['State \t',num2str(j),'\n']));
            fprintf(fid,s1);
            fprintf(fid,s2);
            fprintf(fid,s3);
        end;
    else
    	s1 = strcat(['No States in Phase ',num2str(i),'\n']);
        fprintf(fid,s1);
    end;
    fprintf(fid,'\n');
    if ncontrols>0,
        ulow = limits(i).control.min;
        uupp = limits(i).control.max;
        for j=1:ncontrols
          sulow = num2str(ulow(j));
            suupp = num2str(uupp(j));
            ee = ' <= ';
            sname = strcat(['Control ',num2str(j)]);
            s1 = strcat(['\t During Phase:    \t',sulow,ee,sname,ee,suupp,'\n']);
            fprintf(fid,strcat(['Control \t',num2str(j),'\n']));
            fprintf(fid,s1);
        end;
    else
    	s1 = strcat(['No Controls in Phase ',num2str(i),'\n']);
        fprintf(fid,s1);
    end;
    fprintf(fid,'\n');
    if nparameters>0,
        plow = limits(i).parameter.min;
        pupp = limits(i).parameter.max;
        for j=1:nparameters
            splow = num2str(plow(j));
            spupp = num2str(pupp(j));
            ee = ' <= ';
            sname = strcat(['Parameter ',num2str(j)]);
            s1 = strcat(['\t During Phase:    \t',splow,ee,sname,ee,spupp,'\n']);
            fprintf(fid,strcat(['Parameter \t',num2str(j),'\n']));
            fprintf(fid,s1);
        end;
    else
        s1 = strcat(['No Parameters in Phase ',num2str(i),'\n']);
        fprintf(fid,s1);
    end
    fprintf(fid,'\n');
    if npaths>0,
        pathlow = limits(i).path.min;
        pathupp = limits(i).path.max;
        for j=1:npaths
            spathlow = num2str(pathlow(j));
            spathupp = num2str(pathupp(j));
            ee = ' <= ';
            sname = strcat(['Path ',num2str(j)]);
            s1 = strcat(['\t During Phase:    \t',spathlow,ee,sname,ee,spathupp,'\n']);
            fprintf(fid,strcat(['Path Constraint \t',num2str(j),'\n']));
            fprintf(fid,s1);
        end;
    else
        s1 = strcat(['No Path Constraints in Phase ',num2str(i),'\n']);
        fprintf(fid,s1);
    end
    fprintf(fid,'\n');
    if nevents>0,
        eventlow = limits(i).event.min;
        eventupp = limits(i).event.max;
        for j=1:nevents
            seventlow = num2str(eventlow(j));
            seventupp = num2str(eventupp(j));
            ee = ' <= ';
            sname = strcat(['Event ',num2str(j)]);
            s1 = strcat(['\t\t\t\t',seventlow,ee,sname,ee,seventupp,'\n']);
            fprintf(fid,strcat(['Event Constraint \t',num2str(j),'\n']));
            fprintf(fid,s1);
        end;
    else
        s1 = strcat(['No Event Constraints in Phase ',num2str(i),'\n']);
        fprintf(fid,s1);
    end    
    fprintf(fid,'\n');
    if isfield(limits(i),'duration'),
        durationlow = limits(i).duration.min;
        durationupp = limits(i).duration.max;
        sdurationlow = num2str(durationlow);
        sdurationupp = num2str(durationupp);
        ee = ' <= ';
        s1 = strcat(['Phase Duration: ',sdurationlow,ee,'duration',ee,sdurationupp,'\n']);
        fprintf(fid,s1);
    else
        s1 = strcat(['No Limits on Phase Duration in Phase ',num2str(i),'\n']);
        fprintf(fid,s1);
    end
    fprintf(fid,'\n');
end;
fclose(fid);
