function WriteParameters(tf, Nfe, x, y, theta)
global BV_
delete('BV');
fid = fopen('BV', 'w');
fprintf(fid, '1  %f\r\n', BV_.x0);
fprintf(fid, '2  %f\r\n', BV_.y0);
fprintf(fid, '3  %f\r\n', BV_.theta0);
fprintf(fid, '4  %f\r\n', BV_.v0);
fprintf(fid, '5  %f\r\n', BV_.phy0);
fprintf(fid, '6  %f\r\n', tf);
fprintf(fid, '7  %f\r\n', Nfe);
fprintf(fid, '8  %f\r\n', x);
fprintf(fid, '9  %f\r\n', y);
fprintf(fid, '10  %f\r\n', theta);
fclose(fid);