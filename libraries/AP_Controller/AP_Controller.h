/*
 * AP_Controller.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Controller_H
#define AP_Controller_H

#include <AP_Vector.h>
#include <AP_Var.h>
#include <AP_RcChannel.h>
#include <APM_RC.h>

/// Block
class Block
{
public:
    Block() :
        _input(), _output()
    {
    }
    virtual void update(const float & dt = 0) = 0;
    virtual void connect( Block * block)
    {
    }
    const char * getName() { return _name; }
    const Vector < AP_Var * > & getOutput() const { return _output; }
protected:
    const char * _name;
    Vector< AP_Var * > _input;
    Vector< AP_Var * > _output;
};

/// Servo Block
class ToServo: public Block
{
public:
    ToServo(AP_RcChannel * ch) : _ch(ch)
    {
    }
    virtual void connect(Block * block)
    {
        if (block->getOutput().getSize() > 0)
            _input.push_back(block->getOutput()[0]);
    }
    virtual void update(const float & dt = 0)
    {
        if (_input.getSize() > 0)
            _ch->setPosition(_output[0]);
    }
private:
    float _position;
    AP_RcChannel * _ch;
};

/// SumGain
class SumGain : public Block
{
public:
    /// Constructor that allows 1-8 sum gain pairs, more
    /// can be added if necessary
    SumGain(
        AP_Var * var1,  AP_Var * gain1,
        AP_Var * var2 = NULL, AP_Var * gain2 = NULL,
        AP_Var * var3 = NULL, AP_Var * gain3 = NULL,
        AP_Var * var4 = NULL, AP_Var * gain4 = NULL,
        AP_Var * var5 = NULL, AP_Var * gain5 = NULL,
        AP_Var * var6 = NULL, AP_Var * gain6 = NULL,
        AP_Var * var7 = NULL, AP_Var * gain7 = NULL,
        AP_Var * var8 = NULL, AP_Var * gain8 = NULL) :
        _gain()
    {
        _output.push_back(new AP_Float(0));
        if (var1 && gain1) add(var1,gain1);
        if (var2 && gain2) add(var2,gain2);
        if (var3 && gain3) add(var3,gain3);
        if (var4 && gain4) add(var4,gain4);
        if (var5 && gain5) add(var5,gain5);
        if (var6 && gain6) add(var6,gain6);
        if (var7 && gain7) add(var7,gain7);
        if (var8 && gain8) add(var8,gain8);
    }
    void add(AP_Var * var, AP_Var * gain)
    {
        _input.push_back(var);
        _gain.push_back(gain);
    }
    virtual void connect(Block * block)
    {
        if (block->getOutput().getSize() > 0)
            _input.push_back(block->getOutput()[0]);
    }
    virtual void update(const float & dt = 0)
    {
        if (_output.getSize() < 1) return;
        _output[0]->setF(0);
        for (int i=0;i<_input.getSize();i++)
        {
            _output[0]->setF( _output[i] + _input[i]*_gain[i]);
        }
    }
private:
    Vector< AP_Var * > _gain;
};

/// PID block
class Pid : public Block
{
public:
    Pid(AP_Var::Key key, const prog_char * name,
            float kP = 0.0,
            float kI = 0.0,
            float kD = 0.0,
            float iMax = 0.0,
            uint8_t dFcut = 20.0,
            ) :
        _group(key,name),
        _kP(&_group,0,kP,P_STR("P")), 
        _kI(&_group,0,kP,P_STR("I")), 
        _kD(&_group,0,kP,P_STR("D")), 
        _iMax(&_group,0,kP,P_STR("IMAX")), 
        _fCut(&_group,0,kP,P_STR("FCUT")), 
    {
        _output.push_back(new AP_Float(0));
    }
    virtual void connect(Block * block)
    {
        if (block->getOutput().getSize() > 0)
            _input.push_back(block->getOutput()[0]);
    }
    virtual void update(const float & dt = 0)
    {
        if (_output.getSize() < 1) return;

        // derivative
        float RC = 1/(2*M_PI*_fCut->get()); // low pass filter
        _eD =  _eD + ( ((_e - _input[0]))/dt - _eD ) * (dt / (dt + RC));

        // proportional, note must come after derivative
        // because derivatve uses _e as previous value
        _e = _input[0];

        // integral
        _eI += _e*dt;

        // pid sum
        _output[0]->setF(_kP*_e + _kI*_eI +  _kD*_eD);
    }
private:
    float _e; /// input
    float _eI; /// integral of input
    float _eD; /// derivative of input
    AP_Float * _kP; /// proportional gain
    AP_Float * _kI; /// integral gain
    AP_Float * _kD; /// derivative gain
    AP_Float * _iMax; /// integrator saturation
    AP_Uint8 * _fCut; /// derivative low-pass cut freq (Hz)
};

/// Controller class
class AP_Controller
{
public:
	void addBlock(Block * block)
	{
        if (_blocks.getSize() > 0) 
            _blocks[_blocks.getSize()]->connect(block);
		_blocks.push_back(block);
	}
    void addCh(AP_RcChannel * ch)
	{
		_rc.push_back(ch);
	}
    AP_RcChannel * getRc(int i)
	{
		return _rc[i];
	}
    void update()
    {
        for (int i=0;i<_blocks.getSize();i++)
            _blocks[i]->update();
    }
private:
	Vector<Block * > _blocks;	
    Vector<AP_RcChannel * > _rc;
};

#endif // AP_Controller_H

// vim:ts=4:sw=4:expandtab
