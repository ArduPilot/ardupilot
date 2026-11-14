! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_schedule
    use aerobatics_path
    implicit none
    integer, parameter :: max_bytes=16384,max_commands=256
contains
    subroutine utf8_next(text,p,cp,next,valid)
        character(len=*), intent(in) :: text
        integer, intent(in) :: p
        integer, intent(out) :: cp,next
        logical, intent(out) :: valid
        integer :: first,n,k,b,mincp
        valid=.false.; cp=0; next=p+1
        if (p>len(text)) return
        first=iachar(text(p:p)); n=0; mincp=0
        if (first<128) then
            cp=first; valid=.true.; return
        else if (first>=194 .and. first<=223) then
            n=2; cp=iand(first,31); mincp=128
        else if (first>=224 .and. first<=239) then
            n=3; cp=iand(first,15); mincp=2048
        else if (first>=240 .and. first<=244) then
            n=4; cp=iand(first,7); mincp=65536
        else
            return
        end if
        if (p+n-1>len(text)) return
        do k=1,n-1
            b=iachar(text(p+k:p+k))
            if (b<128 .or. b>191) return
            cp=cp*64+iand(b,63)
        end do
        if (cp<mincp .or. cp>1114111 .or. (cp>=55296 .and. cp<=57343)) return
        next=p+n; valid=.true.
    end subroutine
    logical function whitespace(cp)
        integer, intent(in) :: cp
        whitespace=(cp>=9 .and. cp<=13) .or. cp==32 .or. cp==133 .or. cp==160 .or. cp==5760 .or. &
            (cp>=8192 .and. cp<=8202) .or. cp==8232 .or. cp==8233 .or. cp==8239 .or. cp==8287 .or. cp==12288
    end function
    subroutine trimmed(text,first,last)
        character(len=*), intent(in) :: text
        integer, intent(out) :: first,last
        integer :: p,cp,next
        logical :: valid
        first=1; last=0; p=1
        do while(p<=len(text))
            call utf8_next(text,p,cp,next,valid)
            if (.not.valid) return
            if (.not.whitespace(cp)) then
                if (last==0) first=p
                last=next-1
            end if
            p=next
        end do
    end subroutine
    subroutine next_word(text,p,first,last)
        character(len=*), intent(in) :: text
        integer, intent(inout) :: p
        integer, intent(out) :: first,last
        integer :: cp,next
        logical :: valid
        first=1; last=0
        do while(p<=len(text))
            call utf8_next(text,p,cp,next,valid)
            if (.not.valid) return
            if (.not.whitespace(cp)) exit
            p=next
        end do
        first=p
        do while(p<=len(text))
            call utf8_next(text,p,cp,next,valid)
            if (.not.valid .or. whitespace(cp)) exit
            last=next-1; p=next
        end do
    end subroutine
    logical function alphanumeric(ch)
        character, intent(in) :: ch
        alphanumeric=(ch>='a'.and.ch<='z').or.(ch>='A'.and.ch<='Z').or.(ch>='0'.and.ch<='9')
    end function
    real(c_float) function number(rt,text) result(v)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: text
        type(array_uint8_t_256) :: packet
        type(string_strtofoutput) :: parsed
        integer :: n,k
        integer(c_int32_t) :: code
        v=0.0; n=len(text)
        if (rt%error/=module_ok) return
        if (n<1 .or. n>255) then
            call check(rt,module_invalid); return
        end if
        packet%data=0
        do k=1,n
            if (.not.((text(k:k)>='0'.and.text(k:k)<='9').or.find_char('+- .eE',text(k:k))>0) &
                .or.iachar(text(k:k))==32) then
                call check(rt,module_invalid); return
            end if
            packet%data(k-1)=int(iachar(text(k:k)),c_int8_t)
        end do
        code=string_strtof(rt%ctx,packet,parsed); call check(rt,code)
        if (rt%error/=module_ok) return
        if (int(parsed%consumed)/=n .or. .not.finite(parsed%value)) call check(rt,module_invalid)
        v=parsed%value
    end function
    logical function boolean(rt,text) result(v)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: text
        v=.false.
        if (same_text(text,'true')) then
            v=.true.
        else if (same_text(text,'false')) then
            v=.false.
        else; call check(rt,module_invalid)
        end if
    end function
    subroutine parse_schedule(rt,text,tree,width,eval,build,name,capacity)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: text
        type(path_tree), intent(inout) :: tree
        real(c_float), intent(in) :: width
        type(evaluation), intent(in) :: eval
        logical, intent(in) :: build
        character(len=*), intent(out) :: name
        integer, intent(out) :: capacity
        type(attributes) :: attrs
        character(len=50) :: pending_message
        real(c_float) :: args(4)
        integer :: p,cp,next,offset,ending,first,last,a,b,colon,k,cmd,ncommands,node,root,wordp,vl,vr
        logical :: valid,pending
        name='Trick'; capacity=1; ncommands=0; p=1; pending=.false.; attrs=empty_attributes(); pending_message=''
        if (len(text)>max_bytes) call check(rt,module_invalid)
        do while(p<=len(text) .and. rt%error==module_ok)
            call utf8_next(text,p,cp,next,valid)
            if (.not.valid .or. cp==0) call check(rt,module_invalid)
            p=next
        end do
        if (rt%error/=module_ok) return
        root=0
        if (build) then
            root=new_node(rt,tree,composite); tree%root=root
        end if
        offset=1
        do while (offset<=len(text) .and. rt%error==module_ok)
            ending=find_char(text(offset:),achar(10))
            if (ending==0) then
                ending=len(text)
            else
                ending=offset+ending-2
            end if
            call trimmed(text(offset:ending),first,last)
            first=first+offset-1; last=last+offset-1; offset=ending+2
            if (last<first) cycle
            if (text(first:first)=='#') cycle
            colon=find_char(text(first:last),':')
            if (colon>0) then
                colon=first+colon-1; call trimmed(text(colon+1:last),a,b)
                vl=colon+a; vr=colon+b
                if (same_text(text(first:colon-1),'name')) then
                    if (vr-vl+1<1 .or. vr-vl+1>48) then
                        call check(rt,module_invalid); return
                    end if
                    do k=vl,vr
                        if (.not.alphanumeric(text(k:k)).and.text(k:k)/='_') call check(rt,module_invalid)
                    end do
                    name=text(vl:vr); cycle
                else if (same_text(text(first:colon-1),'message')) then
                    if (vr-vl+1<1 .or. vr-vl+1>50) then
                        call check(rt,module_invalid); return
                    end if
                    do k=vl,vr
                        if (.not.alphanumeric(text(k:k)).and.find_char('_: -'//achar(9),text(k:k))==0) &
                            call check(rt,module_invalid)
                    end do
                    pending_message=text(vl:vr)
                else if (same_text(text(first:colon-1),'roll_ref')) then
                    attrs%have_roll=.true.; attrs%roll_ref=number(rt,text(vl:vr))
                else if (same_text(text(first:colon-1),'thr_boost')) then
                    attrs%have_boost=.true.; attrs%thr_boost=boolean(rt,text(vl:vr))
                else if (same_text(text(first:colon-1),'pos_corr')) then
                    attrs%have_corr=.true.; attrs%pos_corr=number(rt,text(vl:vr))
                else if (same_text(text(first:colon-1),'pos_gain_mul')) then
                    attrs%have_gain=.true.; attrs%pos_gain_mul=number(rt,text(vl:vr))
                else if (same_text(text(first:colon-1),'shift_xy')) then
                    attrs%have_shift=.true.; attrs%shift_xy=boolean(rt,text(vl:vr))
                else
                    call check(rt,module_invalid)
                end if
                pending=.true.; cycle
            end if
            wordp=1; call next_word(text(first:last),wordp,a,b)
            cmd=command_named(text(first+a-1:first+b-1)); args=0.0
            if (cmd==0) then
                call check(rt,module_invalid); return
            end if
            k=0
            do
                call next_word(text(first:last),wordp,a,b)
                if (b<a) exit
                k=k+1
                if (k>4) then
                    call check(rt,module_invalid); return
                end if
                args(k)=number(rt,text(first+a-1:first+b-1))
            end do
            ncommands=ncommands+1; capacity=capacity+command_nodes(cmd)
            if (ncommands>max_commands) call check(rt,module_invalid)
            if (rt%error/=module_ok) return
            if (build) then
                node=build_maneuver(rt,tree,cmd,args,width,eval)
                if (node==0 .or. rt%error/=module_ok) return
                tree%nodes(node)%attrs=attrs; tree%nodes(node)%rebuild=.true.
                tree%nodes(node)%message_length=text_length(pending_message)
                do k=1,tree%nodes(node)%message_length
                    tree%nodes(node)%message(k)=pending_message(k:k)
                end do
                call append_node(tree,root,node)
            end if
            attrs=empty_attributes(); pending_message=''; pending=.false.
        end do
        if (ncommands==0 .or. pending) call check(rt,module_invalid)
        if (build) call compose(rt,tree,root,width,eval)
    end subroutine
    subroutine load_schedule(rt,filename,tree,width,eval,found,name,fingerprint)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: filename
        type(path_tree), intent(inout) :: tree
        real(c_float), intent(in) :: width
        type(evaluation), intent(in) :: eval
        logical, intent(out) :: found
        character(len=*), intent(out) :: name
        integer(c_int64_t), intent(out), optional :: fingerprint
        integer(c_int64_t) :: hash
        type(c_ptr) :: memory
        character(len=max_bytes), pointer :: text
        type(array_uint8_t_256) :: path
        type(optional_file_read_atoutput) :: chunk
        integer(c_int32_t) :: code
        integer :: n,k,received,capacity
        found=.false.; name=''; n=0
        if (present(fingerprint)) fingerprint=0
        if (rt%error/=module_ok) return
        memory=arena_allocate(int(max_bytes,c_size_t))
        if (.not.c_associated(memory)) then
            call check(rt,module_no_memory); return
        end if
        call c_f_pointer(memory,text); path%data=0
        do k=1,len(filename)
            path%data(k-1)=int(iachar(filename(k:k)),c_int8_t)
        end do
        do while(rt%error==module_ok)
            code=file_read_at(rt%ctx,path,int(n,c_int32_t),256_c_int16_t,chunk); call check(rt,code)
            if (rt%error/=module_ok) exit
            if (.not.chunk%present) then
                if (n/=0) call check(rt,module_host_error)
                exit
            end if
            found=.true.; received=int(chunk%value%length)
            if (received==0) exit
            if (received<0 .or. received>256 .or. n+received>max_bytes) then
                call check(rt,module_invalid); exit
            end if
            do k=0,received-1
                text(n+k+1:n+k+1)=achar(iand(int(chunk%value%data%data(k)),255))
            end do
            n=n+received
        end do
        if (found .and. rt%error==module_ok) then
            hash=2166136261_c_int64_t
            do k=1,n
                hash=iand(ieor(hash,int(iachar(text(k:k)),c_int64_t))*16777619_c_int64_t,4294967295_c_int64_t)
            end do
            if (present(fingerprint)) fingerprint=hash
            call parse_schedule(rt,text(1:n),tree,width,eval,.false.,name,capacity)
            if (rt%error==module_ok) call allocate_tree(rt,tree,capacity)
            if (rt%error==module_ok) call parse_schedule(rt,text(1:n),tree,width,eval,.true.,name,capacity)
        end if
        call arena_free(memory)
    end subroutine
end module aerobatics_schedule
