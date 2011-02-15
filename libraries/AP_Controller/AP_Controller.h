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

#include <AP_Common.h>
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
    virtual void update(const float & dt) = 0;
    virtual void connect( Block * block)
    {
    }
    const char * getName() { return _name; }
    const Vector < AP_Float * > & getOutput() const { return _output; }
protected:
    const char * _name;
    Vector< const AP_Float * > _input;
    Vector< AP_Float * > _output;
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
        {
            _ch->setNormalized(_input[0]->get());
        }
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
            AP_Float & var1 = AP_Float_zero, AP_Float & gain1 = AP_Float_zero,
            AP_Float & var2 = AP_Float_zero, AP_Float & gain2 = AP_Float_zero,
            AP_Float & var3 = AP_Float_zero, AP_Float & gain3 = AP_Float_zero,
            AP_Float & var4 = AP_Float_zero, AP_Float & gain4 = AP_Float_zero,
            AP_Float & var5 = AP_Float_zero, AP_Float & gain5 = AP_Float_zero,
            AP_Float & var6 = AP_Float_zero, AP_Float & gain6 = AP_Float_zero,
            AP_Float & var7 = AP_Float_zero, AP_Float & gain7 = AP_Float_zero,
            AP_Float & var8 = AP_Float_zero, AP_Float & gain8 = AP_Float_zero)
    {
        if ( (&var1 != &AP_Float_zero) && (&gain1 != &AP_Float_zero) ) add(var1,gain1);
        if ( (&var2 != &AP_Float_zero) && (&gain2 != &AP_Float_zero) ) add(var2,gain2);
        if ( (&var3 != &AP_Float_zero) && (&gain3 != &AP_Float_zero) ) add(var3,gain3);
        if ( (&var4 != &AP_Float_zero) && (&gain4 != &AP_Float_zero) ) add(var4,gain4);
        if ( (&var5 != &AP_Float_zero) && (&gain5 != &AP_Float_zero) ) add(var5,gain5);
        if ( (&var6 != &AP_Float_zero) && (&gain6 != &AP_Float_zero) ) add(var6,gain6);
        if ( (&var7 != &AP_Float_zero) && (&gain7 != &AP_Float_zero) ) add(var7,gain7);
        if ( (&var8 != &AP_Float_zero) && (&gain8 != &AP_Float_zero) ) add(var8,gain8);

        // create output
        _output.push_back(new AP_Float(0));
    }
    void add(AP_Float & var, AP_Float & gain)
    {
        _input.push_back(&var);
        _gain.push_back(&gain);
    }
    virtual void update(const float & dt = 0)
    {
        if (_output.getSize() < 1) return;
        float sum =0;
        for (int i=0;i<_input.getSize();i++) sum += _input[i]->get() * _gain[i]->get() ;
        _output[0]->set(sum);
    }
private:
    Vector< AP_Float * > _gain;
};

/// PID block
class Pid : public Block, public AP_Var_group
{
public:
    Pid(AP_Var::Key key, const prog_char * name,
            float kP = 0.0,
            float kI = 0.0,
            float kD = 0.0,
            float iMax = 0.0,
            uint8_t dFcut = 20.0
            ) :
        AP_Var_group(key,name),
        _kP(this,1,kP,PSTR("P")), 
        _kI(this,2,kI,PSTR("I")), 
        _kD(this,3,kD,PSTR("D")), 
        _iMax(this,4,iMax,PSTR("IMAX")), 
        _fCut(this,5,dFcut,PSTR("FCUT"))
    {
        _output.push_back(new AP_Float(0));
    }
    virtual void connect(Block * block)
    {
        if (!block) return;
        if (block->getOutput().getSize() > 0)
            _input.push_back(block->getOutput()[0]);
    }
    virtual void update(const float & dt)
    {
        if (_output.getSize() < 1 || (!_input[0]) || (!_output[0]) ) return;

        // derivative with low pass
        float RC = 1/(2*M_PI*_fCut); // low pass filter
        _eD =  _eD + ( ( _eP - _input[0]->get() )/dt - _eD ) * (dt / (dt + RC));

        // proportional, note must come after derivative
        // because derivatve uses _eP as previous value
        _eP = _input[0]->get();

        // integral
        _eI += _eP*dt;

        // wind up guard
        if (_eI > _iMax) _eI = _iMax;
        else if (_eI < -_iMax) _eI = -_iMax;

        // pid sum
        _output[0]->set(_kP*_eP + _kI*_eI +  _kD*_eD);

        // debug output
        /*Serial.println("kP, kI, kD: ");
        Serial.print(_kP,5); Serial.print(" ");
        Serial.print(_kI,5); Serial.print(" ");
        Serial.println(_kD,5);
        Serial.print("eP, eI, eD: ");
        Serial.print(_eP,5); Serial.print(" ");
        Serial.print(_eI,5); Serial.print(" ");
        Serial.println(_eD,5);
        Serial.print("input: ");
        Serial.println(_input[0]->get(),5);
        Serial.print("output: ");
        Serial.println(_output[0]->get(),5);*/
    }
private:
    float _eP; /// input
    float _eI; /// integral of input
    float _eD; /// derivative of input
    AP_Float _kP; /// proportional gain
    AP_Float _kI; /// integral gain
    AP_Float _kD; /// derivative gain
    AP_Float _iMax; /// integrator saturation
    AP_Uint8 _fCut; /// derivative low-pass cut freq (Hz)
};

/// Controller class
class AP_Controller
{
public:
    void addBlock(Block * block)
    {
        if (_blocks.getSize() > 0) 
            block->connect(_blocks[_blocks.getSize()-1]);
        _blocks.push_back(block);
    }
    void update(const double dt)
    {
        for (int i=0;i<_blocks.getSize();i++)
        {
            if (!_blocks[i]) continue;
            _blocks[i]->update(dt);
        }
    }
private:
    Vector<Block * > _blocks;   
};

#endif // AP_Controller_H

// vim:ts=4:sw=4:expandtab
