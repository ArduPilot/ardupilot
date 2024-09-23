const ftype PS0 = cosf(dirn);
const ftype PS1 = powf(PS0, 2);
const ftype PS2 = sinf(dirn);
const ftype PS3 = powf(PS2, 2);
const ftype PS4 = dirnVar*powf(spd, 2);
const ftype PS5 = PS0*PS2*(-PS4 + spdVar);


R_obs[0][0] = PS1*spdVar + PS3*PS4;
R_obs[1][0] = PS5;
R_obs[0][1] = PS5;
R_obs[1][1] = PS1*PS4 + PS3*spdVar;


