clc; close all; clear all;



%% Insect-like algorithm

load house % importing a map of a house
about house % type of variable, and how much it occupies
place % Structure that contains all the spaces of the house
place.br3 %bedroom 3, coordinates of the center of that space in the house.
place.kitchen
al_insecto = Bug2(house) %creating an instance of the bug2 class
al_insecto.plot() %Drawing the loaded map.
%%al_insecto.query(place.br3,place.br2,'animate')
P= al_insecto.query(place.br3,place.br2); hold on; plot(P(:,1),P(:,2),'blue');
P = al_insecto.query([],[],'animate')

%% Distance transform 

im = zeros(26,26);% We create virgin matrix without obstacles
im(16:21, 5:17) = 1; % We create obstacle
imshow(1-im) % show in white color 
DT = bwdist(im) % calculate distance transform
image(DT,'CDataMapping','scaled')

dx = DXform(house) %Object type DXForm
dx.plan(place.kitchen) %Create a plane, based on distance to reach that goal. 
dx.plot()
p = dx.query(place.br3,place.kitchen,'animate')
dx.plot(p)


