function [contact_pt,hit_pt,error] = Get_hit_cont_pts(ROBOT,WORKSPACE,...
    prev_contact_pt,r,guess)
done = false;
trial = 0; %Trials to shift the robot around
while(~done)
    done = true; %Shouldn't need to loop around
    %Returns the current contact point (with the surface) along with the
    %current hit point (if such hit point exists) as well as with a boolean
    %type error variable
    
    error = 0; %If all goes well, there is no error.
    num_contacts = 1; %The 'theoretical' number of contact points (1 or 2)
    
    %Because of the irregularities in the bitmaps (one cannot make a
    %perfect sphere out of cubes), there might be a few contact points
    %where in theory (with a perfect sphere) there would only be one; hence
    %the distinction between num_contacts (the 'theoretical' number of 
    %contact points) and the true number of contact points found.
    [contacts_x,contacts_y,contacts_z] = ind2sub(size(ROBOT),find(ROBOT ...
        & WORKSPACE));
    if(length(contacts_x)~=length(contacts_y) || ...
            length(contacts_y)~=length(contacts_z) ...
            || length(contacts_z)~=length(contacts_x))
        fprinf('WARNING: vectors length differ in contact matrix\n');
    end
    %Vector length, and tolerance level of spacing beween points that 
    %should in theory be touching each other
    len = length(contacts_x); delta = r/10;
    %Checking that there is at least one contact point (the robot is not 
    %flying around).
    %If we get unlucky and the sphere does not land right on the surface,
    %we can try and move it very slightly.
    if(len < 1)
        if(trial > 0)
            fprintf('ERROR: contact has been lost\n');
            contact_pt = zeros(3,1); hit_pt = zeros(3,1); error = 1;
            return
        end
        delta_move = round(r/10);
        finished = false;
        for i=0:delta_move
            for j=0:delta_move
                for k=0:delta_move
                    tmp_ROBOT = circshift(ROBOT,[i, j, k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[-i, j, k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        i = -i;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[i, -j, k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        j = -j;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[i, j, -k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        k = -k;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[i, -j, -k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        j = -j; k = -k;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[-i, j, -k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        i = -i; k = -k;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[-i, -j, k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        i = -i; j = -j;
                        break;
                    end
                    tmp_ROBOT = circshift(ROBOT,[-i, -j, -k]);
                    tmp_index = find(tmp_ROBOT & WORKSPACE,1);
                    if(~isempty(tmp_index))
                        finished = true;
                        i = -i; j = -j; k = -k;
                        break;
                    end
                end
                if(finished); break; end;
            end
            if(finished); break; end;
        end
        if(~finished);
            fprintf('ERROR: No contact has been found within range\n');
            contact_pt = zeros(3,1); hit_pt = zeros(3,1); error = 1;
            return
        end
        shift_x = i;
        shift_y = j;
        shift_z = k;
        ROBOT = circshift(ROBOT,[shift_x, shift_y, shift_z]);
        done = false;
        trial = trial + 1;
    else
        jump_index = 0; %To track where the discontinuity is located
        for i=1:len-1
            if(abs(contacts_x(i+1)-contacts_x(i)) > delta ...
                    && abs(contacts_y(i+1)-contacts_y(i)) > delta ...
                    && abs(contacts_z(i+1)-contacts_z(i)) > delta)
                jump_index = i;
                num_contacts = num_contacts + 1; %2 contacts/hit
            end
        end
        %Checking that there are no more than 2 theoretical contact points:
        if(num_contacts > 2)
            fprintf('ERROR: there are more than 2 theoretical contact points\n');
            fprintf('\t\t num_contacts = %i\n',num_contacts);
            contact_pt = zeros(3,1); hit_pt = zeros(3,1); error = 1;
            return
        end
        %There are two cases: either there is 1 theoretical contact point 
        %or there are two. Those must be taken care of separately.
        if(num_contacts == 1) %Only 1 contact point. Note: jump_index == 0
            if(~strcmp(guess,'NaN') ...
                    && WORKSPACE(guess(1),guess(2),guess(3)) ...
                    && ROBOT(guess(1),guess(2),guess(3)))
                contact_pt = guess;
            else
                %Where the theoretical contact point is located in the
                %contact vectors:
                contact_index = ceil(len/2);
                contact_pt = [contacts_x(contact_index); ...
                   contacts_y(contact_index); contacts_z(contact_index)];
            end
            hit_pt = 'NaN';
        else %2 contact points (more complicated because I need to
            %differentiate between the contact vectors and the hit vectors
            
            %Making sure a second contact point is allowed:
            if(strcmp(prev_contact_pt,'NaN'))
                fprintf('ERROR: there cannot be a second contact point at this point \n');
                contact_pt = zeros(3,1); hit_pt = zeros(3,1); error = 1;
                return
            end
            
            %The first part of the contacts vectors
            contacts_x1 = contacts_x(1:jump_index);
            contacts_y1 = contacts_y(1:jump_index);
            contacts_z1 = contacts_z(1:jump_index);
            len1 = length(contacts_x1);
            contact_index1 = ceil(len1/2);
            contact_pt1 = [contacts_x1(contact_index1); ...
                contacts_y1(contact_index1); contacts_z1(contact_index1)];
            %The second part of the contacts vectors
            contacts_x2 = contacts_x(jump_index+1:end);
            contacts_y2 = contacts_y(jump_index+1:end);
            contacts_z2 = contacts_z(jump_index+1:end);
            len2 = length(contacts_x2);
            contact_index2 = ceil(len2/2);
            contact_pt2 = [contacts_x2(contact_index2); ...
                contacts_y2(contact_index2); contacts_z2(contact_index2)];
            %Find which contact_pt is the real one and which is the hit_pt
            d1 = sum((prev_contact_pt-contact_pt1').^2);
            d2 = sum((prev_contact_pt-contact_pt2').^2);
            if(d1<d2) %contact_pt1 is closer to the previous contact point)
                contact_pt = contact_pt1;
                hit_pt = contact_pt2;
            else
                contact_pt = contact_pt2;
                hit_pt = contact_pt1;
            end
        end
    end
end
end

