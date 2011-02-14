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


#define APVarPtr2Float(var) (*(AP_Meta_class::meta_cast<AP_Float>((AP_Var *)var)))

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
    Vector< const AP_Var * > _input;
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
            _ch->setNormalized(APVarPtr2Float(_input[0]));
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
            AP_Var & var1 = AP_Float_zero, AP_Var & gain1 = AP_Float_zero,
            AP_Var & var2 = AP_Float_zero, AP_Var & gain2 = AP_Float_zero,
            AP_Var & var3 = AP_Float_zero, AP_Var & gain3 = AP_Float_zero,
            AP_Var & var4 = AP_Float_zero, AP_Var & gain4 = AP_Float_zero,
            AP_Var & var5 = AP_Float_zero, AP_Var & gain5 = AP_Float_zero,
            AP_Var & var6 = AP_Float_zero, AP_Var & gain6 = AP_Float_zero,
            AP_Var & var7 = AP_Float_zero, AP_Var & gain7 = AP_Float_zero,
            AP_Var & var8 = AP_Float_zero, AP_Var & gain8 = AP_Float_zero)
    {
        if ( (&var1 == &AP_Float_zero) || (&gain1 == &AP_Float_zero) ) add(var1,gain1);
        if ( (&var2 == &AP_Float_zero) || (&gain2 == &AP_Float_zero) ) add(var2,gain2);
        if ( (&var3 == &AP_Float_zero) || (&gain3 == &AP_Float_zero) ) add(var3,gain3);
        if ( (&var4 == &AP_Float_zero) || (&gain4 == &AP_Float_zero) ) add(var4,gain4);
        if ( (&var5 == &AP_Float_zero) || (&gain5 == &AP_Float_zero) ) add(var5,gain5);
        if ( (&var6 == &AP_Float_zero) || (&gain6 == &AP_Float_zero) ) add(var6,gain6);
        if ( (&var7 == &AP_Float_zero) || (&gain7 == &AP_Float_zero) ) add(var7,gain7);
        if ( (&var8 == &AP_Float_zero) || (&gain8 == &AP_Float_zero) ) add(var8,gain8);

        // create output
        _output.push_back(new AP_Float(0));
    }
    void add(AP_Var & var, AP_Var & gain)
    {
        _input.push_back(&var);
        _gain.push_back(&gain);
    }
    virtual void update(const float & dt = 0)
    {
        if (_output.getSize() < 1) return;
        _output[0]=0;
        for (int i=0;i<_input.getSize();i++)
        {
            *_output[0] = APVarPtr2Float(_output[i]) * APVarPtr2Float(_gain[i]) ;
        }
    }
private:
    Vector< AP_Var * > _gain;
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
    virtual void update(const float & dt = 0)
    {
        if (_output.getSize() < 1 || !_input[0] || !_output[0]) return;

        // derivative
        float RC = 1/(2*M_PI*_fCut); // low pass filter
        _eD =  _eD + ( ((_e - APVarPtr2Float(_input[0])))/dt - _eD ) * (dt / (dt + RC));

        // proportional, note must come after derivative
        // because derivatve uses _e as previous value
        _e = APVarPtr2Float(_input[0]);

        // integral
        _eI += _e*dt;

        // pid sum
        *_output[0] = _kP*_e + _kI*_eI +  _kD*_eD;


        Serial.println("debug");
        Serial.println(_kP);
        Serial.println(_kI);
        Serial.println(_kD);
        Serial.println(_e);
        Serial.println(_eI);
        Serial.println(_eD);
        Serial.println(APVarPtr2Float(_output[0]));


    }
private:
    float _e; /// input
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
            _blocks[_blocks.getSize()-1]->connect(block);
        _blocks.push_back(block);
    }
    void update(const double dt=0)
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
