; Listing generated by Microsoft (R) Optimizing Compiler Version 19.00.24213.1 

	TITLE	E:\Documents\Cours\Git\Raven\Common\misc\Cgdi.cpp
	.686P
	.XMM
	include listing.inc
	.model	flat

INCLUDELIB LIBCMTD
INCLUDELIB OLDNAMES

CONST	SEGMENT
?colors@@3QBKB DD 0ffH					; colors
	DD	0ff0000H
	DD	0ff00H
	DD	00H
	DD	0c8c8ffH
	DD	0c8c8c8H
	DD	0ffffH
	DD	0aaffH
	DD	0aa00ffH
	DD	05a85H
	DD	0ffffffH
	DD	06400H
	DD	0ffff00H
	DD	0c8c8c8H
	DD	0e6e6ffH
CONST	ENDS
PUBLIC	?__empty_global_delete@@YAXPAX@Z		; __empty_global_delete
PUBLIC	?__empty_global_delete@@YAXPAXI@Z		; __empty_global_delete
PUBLIC	?max@?$numeric_limits@H@std@@SAHXZ		; std::numeric_limits<int>::max
PUBLIC	?min@?$numeric_limits@M@std@@SAMXZ		; std::numeric_limits<float>::min
PUBLIC	?max@?$numeric_limits@M@std@@SAMXZ		; std::numeric_limits<float>::max
PUBLIC	?min@?$numeric_limits@N@std@@SANXZ		; std::numeric_limits<double>::min
PUBLIC	?max@?$numeric_limits@N@std@@SANXZ		; std::numeric_limits<double>::max
PUBLIC	??0Cgdi@@AAE@XZ					; Cgdi::Cgdi
PUBLIC	??1Cgdi@@QAE@XZ					; Cgdi::~Cgdi
PUBLIC	?Instance@Cgdi@@SAPAV1@XZ			; Cgdi::Instance
PUBLIC	__real@0010000000000000
PUBLIC	__real@00800000
PUBLIC	__real@7f7fffff
PUBLIC	__real@7fefffffffffffff
EXTRN	_atexit:PROC
EXTRN	__Init_thread_header:PROC
EXTRN	__Init_thread_abort:PROC
EXTRN	__Init_thread_footer:PROC
EXTRN	__imp__CreatePen@12:PROC
EXTRN	__imp__CreateSolidBrush@4:PROC
EXTRN	__imp__DeleteObject@4:PROC
EXTRN	@__security_check_cookie@4:PROC
EXTRN	__RTC_CheckEsp:PROC
EXTRN	__RTC_InitBase:PROC
EXTRN	__RTC_Shutdown:PROC
EXTRN	___CxxFrameHandler3:PROC
EXTRN	__Init_thread_epoch:DWORD
EXTRN	___security_cookie:DWORD
EXTRN	__fltused:DWORD
EXTRN	__tls_array:DWORD
EXTRN	__tls_index:DWORD
_BSS	SEGMENT
?MaxInt@@3HB DD	01H DUP (?)				; MaxInt
	ALIGN	8

?MaxDouble@@3NB DQ 01H DUP (?)				; MaxDouble
?MinDouble@@3NB DQ 01H DUP (?)				; MinDouble
?MaxFloat@@3MB DD 01H DUP (?)				; MaxFloat
?MinFloat@@3MB DD 01H DUP (?)				; MinFloat
_BSS	ENDS
;	COMDAT ?instance@?1??Instance@Cgdi@@SAPAV2@XZ@4V2@A
_BSS	SEGMENT
?instance@?1??Instance@Cgdi@@SAPAV2@XZ@4V2@A DB 080H DUP (?) ; `Cgdi::Instance'::`2'::instance
_BSS	ENDS
;	COMDAT ?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA
_BSS	SEGMENT
?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA DD 01H DUP (?)	; TSS0<`template-parameter-2',Cgdi::stance,Cgdi * * const volatile,void,int, ?? &>
_BSS	ENDS
CRT$XCU	SEGMENT
?MaxInt$initializer$@@3P6AXXZA DD FLAT:??__EMaxInt@@YAXXZ ; MaxInt$initializer$
CRT$XCU	ENDS
;	COMDAT __real@7fefffffffffffff
CONST	SEGMENT
__real@7fefffffffffffff DQ 07fefffffffffffffr	; 1.79769e+308
CONST	ENDS
;	COMDAT __real@7f7fffff
CONST	SEGMENT
__real@7f7fffff DD 07f7fffffr			; 3.40282e+38
CONST	ENDS
;	COMDAT __real@00800000
CONST	SEGMENT
__real@00800000 DD 000800000r			; 1.17549e-38
CONST	ENDS
;	COMDAT __real@0010000000000000
CONST	SEGMENT
__real@0010000000000000 DQ 00010000000000000r	; 2.22507e-308
CONST	ENDS
;	COMDAT rtc$TMZ
rtc$TMZ	SEGMENT
__RTC_Shutdown.rtc$TMZ DD FLAT:__RTC_Shutdown
rtc$TMZ	ENDS
;	COMDAT rtc$IMZ
rtc$IMZ	SEGMENT
__RTC_InitBase.rtc$IMZ DD FLAT:__RTC_InitBase
rtc$IMZ	ENDS
xdata$x	SEGMENT
__unwindtable$?Instance@Cgdi@@SAPAV1@XZ DD 0ffffffffH
	DD	FLAT:__unwindfunclet$?Instance@Cgdi@@SAPAV1@XZ$0
__ehfuncinfo$?Instance@Cgdi@@SAPAV1@XZ DD 019930522H
	DD	01H
	DD	FLAT:__unwindtable$?Instance@Cgdi@@SAPAV1@XZ
	DD	2 DUP(00H)
	DD	2 DUP(00H)
	DD	00H
	DD	01H
xdata$x	ENDS
CRT$XCU	SEGMENT
?MaxDouble$initializer$@@3P6AXXZA DD FLAT:??__EMaxDouble@@YAXXZ ; MaxDouble$initializer$
CRT$XCU	ENDS
CRT$XCU	SEGMENT
?MinDouble$initializer$@@3P6AXXZA DD FLAT:??__EMinDouble@@YAXXZ ; MinDouble$initializer$
CRT$XCU	ENDS
CRT$XCU	SEGMENT
?MaxFloat$initializer$@@3P6AXXZA DD FLAT:??__EMaxFloat@@YAXXZ ; MaxFloat$initializer$
CRT$XCU	ENDS
CRT$XCU	SEGMENT
?MinFloat$initializer$@@3P6AXXZA DD FLAT:??__EMinFloat@@YAXXZ ; MinFloat$initializer$
CRT$XCU	ENDS
; Function compile flags: /Odtp /RTCsu
;	COMDAT ??__Finstance@?1??Instance@Cgdi@@SAPAV1@XZ@YAXXZ
text$yd	SEGMENT
??__Finstance@?1??Instance@Cgdi@@SAPAV1@XZ@YAXXZ PROC	; `Cgdi::Instance'::`2'::`dynamic atexit destructor for 'instance'', COMDAT
	push	ebp
	mov	ebp, esp
	mov	ecx, OFFSET ?instance@?1??Instance@Cgdi@@SAPAV2@XZ@4V2@A
	call	??1Cgdi@@QAE@XZ				; Cgdi::~Cgdi
	cmp	ebp, esp
	call	__RTC_CheckEsp
	pop	ebp
	ret	0
??__Finstance@?1??Instance@Cgdi@@SAPAV1@XZ@YAXXZ ENDP	; `Cgdi::Instance'::`2'::`dynamic atexit destructor for 'instance''
text$yd	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\cgdi.cpp
_TEXT	SEGMENT
__$EHRec$ = -12						; size = 12
?Instance@Cgdi@@SAPAV1@XZ PROC				; Cgdi::Instance

; 9    : {

	push	ebp
	mov	ebp, esp
	push	-1
	push	__ehhandler$?Instance@Cgdi@@SAPAV1@XZ
	mov	eax, DWORD PTR fs:0
	push	eax
	mov	eax, DWORD PTR ___security_cookie
	xor	eax, ebp
	push	eax
	lea	eax, DWORD PTR __$EHRec$[ebp]
	mov	DWORD PTR fs:0, eax

; 10   :   static Cgdi instance;

	mov	eax, DWORD PTR __tls_index
	mov	ecx, DWORD PTR fs:__tls_array
	mov	edx, DWORD PTR [ecx+eax*4]
	mov	eax, DWORD PTR ?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA
	cmp	eax, DWORD PTR __Init_thread_epoch[edx]
	jle	SHORT $LN2@Instance
	push	OFFSET ?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA
	call	__Init_thread_header
	add	esp, 4
	cmp	DWORD PTR ?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA, -1
	jne	SHORT $LN2@Instance
	mov	DWORD PTR __$EHRec$[ebp+8], 0
	mov	ecx, OFFSET ?instance@?1??Instance@Cgdi@@SAPAV2@XZ@4V2@A
	call	??0Cgdi@@AAE@XZ				; Cgdi::Cgdi
	push	OFFSET ??__Finstance@?1??Instance@Cgdi@@SAPAV1@XZ@YAXXZ ; `Cgdi::Instance'::`2'::`dynamic atexit destructor for 'instance''
	call	_atexit
	add	esp, 4
	mov	DWORD PTR __$EHRec$[ebp+8], -1
	push	OFFSET ?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA
	call	__Init_thread_footer
	add	esp, 4
$LN2@Instance:

; 11   :   return &instance;

	mov	eax, OFFSET ?instance@?1??Instance@Cgdi@@SAPAV2@XZ@4V2@A

; 12   : }

	mov	ecx, DWORD PTR __$EHRec$[ebp]
	mov	DWORD PTR fs:0, ecx
	pop	ecx
	add	esp, 12					; 0000000cH
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
_TEXT	ENDS
text$x	SEGMENT
__unwindfunclet$?Instance@Cgdi@@SAPAV1@XZ$0:
	push	OFFSET ?$TSS0@?1??Instance@Cgdi@@SAPAV2@XZ@4HA
	call	__Init_thread_abort
	pop	ecx
	ret	0
__ehhandler$?Instance@Cgdi@@SAPAV1@XZ:
	mov	edx, DWORD PTR [esp+8]
	lea	eax, DWORD PTR [edx+12]
	mov	ecx, DWORD PTR [edx-4]
	xor	ecx, eax
	call	@__security_check_cookie@4
	mov	eax, OFFSET __ehfuncinfo$?Instance@Cgdi@@SAPAV1@XZ
	jmp	___CxxFrameHandler3
text$x	ENDS
?Instance@Cgdi@@SAPAV1@XZ ENDP				; Cgdi::Instance
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\cgdi.cpp
_TEXT	SEGMENT
_this$ = -4						; size = 4
??1Cgdi@@QAE@XZ PROC					; Cgdi::~Cgdi
; _this$ = ecx

; 54   : {

	push	ebp
	mov	ebp, esp
	push	ecx
	push	esi
	mov	DWORD PTR [ebp-4], -858993460		; ccccccccH
	mov	DWORD PTR _this$[ebp], ecx

; 55   :   DeleteObject(m_BlackPen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+4]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 56   :   DeleteObject(m_WhitePen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+8]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 57   :   DeleteObject(m_RedPen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+12]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 58   :   DeleteObject(m_GreenPen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+16]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 59   :   DeleteObject(m_BluePen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+20]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 60   :   DeleteObject(m_GreyPen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+24]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 61   :   DeleteObject(m_PinkPen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+28]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 62   :   DeleteObject(m_OrangePen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+32]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 63   :   DeleteObject(m_YellowPen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+36]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 64   :   DeleteObject(m_PurplePen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+40]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 65   :   DeleteObject(m_BrownPen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+44]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 66   :   DeleteObject(m_OldPen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 67   :   
; 68   :   DeleteObject(m_DarkGreenPen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+48]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 69   : 
; 70   :   DeleteObject(m_LightBluePen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+52]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 71   :   DeleteObject(m_LightGreyPen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+56]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 72   :   DeleteObject(m_LightPinkPen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+60]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 73   :   
; 74   :   DeleteObject(m_ThickBlackPen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+64]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 75   :   DeleteObject(m_ThickWhitePen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+68]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 76   :   DeleteObject(m_ThickRedPen);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+72]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 77   :   DeleteObject(m_ThickGreenPen);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+76]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 78   :   DeleteObject(m_ThickBluePen);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+80]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 79   : 
; 80   :   DeleteObject(m_GreenBrush);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+92]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 81   :   DeleteObject(m_RedBrush);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+88]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 82   :   DeleteObject(m_BlueBrush);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+96]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 83   :   DeleteObject(m_OldBrush);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+84]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 84   :   DeleteObject(m_GreyBrush);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+100]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 85   :   DeleteObject(m_BrownBrush);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+104]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 86   :   DeleteObject(m_LightBlueBrush);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+116]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 87   :   DeleteObject(m_YellowBrush);

	mov	esi, esp
	mov	edx, DWORD PTR _this$[ebp]
	mov	eax, DWORD PTR [edx+108]
	push	eax
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 88   :   DeleteObject(m_DarkGreenBrush);

	mov	esi, esp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	edx, DWORD PTR [ecx+120]
	push	edx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 89   :   DeleteObject(m_OrangeBrush);

	mov	esi, esp
	mov	eax, DWORD PTR _this$[ebp]
	mov	ecx, DWORD PTR [eax+112]
	push	ecx
	call	DWORD PTR __imp__DeleteObject@4
	cmp	esi, esp
	call	__RTC_CheckEsp

; 90   : 
; 91   : }

	pop	esi
	add	esp, 4
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
??1Cgdi@@QAE@XZ ENDP					; Cgdi::~Cgdi
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\cgdi.cpp
_TEXT	SEGMENT
_this$ = -4						; size = 4
??0Cgdi@@AAE@XZ PROC					; Cgdi::Cgdi
; _this$ = ecx

; 15   : {

	push	ebp
	mov	ebp, esp
	push	ecx
	push	esi
	mov	DWORD PTR [ebp-4], -858993460		; ccccccccH
	mov	DWORD PTR _this$[ebp], ecx

; 16   :   m_BlackPen = CreatePen(PS_SOLID, 1, colors[black]);

	mov	eax, 4
	imul	ecx, eax, 3
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+4], eax

; 17   :   m_WhitePen = CreatePen(PS_SOLID, 1, colors[white]);

	mov	edx, 4
	imul	eax, edx, 10
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+8], eax

; 18   :   m_RedPen = CreatePen(PS_SOLID, 1, colors[red]);

	mov	eax, 4
	imul	ecx, eax, 0
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+12], eax

; 19   :   m_GreenPen = CreatePen(PS_SOLID, 1, colors[green]);

	mov	edx, 4
	shl	edx, 1
	mov	esi, esp
	mov	eax, DWORD PTR ?colors@@3QBKB[edx]
	push	eax
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+16], eax

; 20   :   m_BluePen = CreatePen(PS_SOLID, 1, colors[blue]);

	mov	edx, 4
	shl	edx, 0
	mov	esi, esp
	mov	eax, DWORD PTR ?colors@@3QBKB[edx]
	push	eax
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+20], eax

; 21   :   m_GreyPen = CreatePen(PS_SOLID, 1, colors[grey]);

	mov	edx, 4
	imul	eax, edx, 5
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+24], eax

; 22   :   m_PinkPen = CreatePen(PS_SOLID, 1, colors[pink]);

	mov	eax, 4
	shl	eax, 2
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+28], eax

; 23   :   m_YellowPen = CreatePen(PS_SOLID, 1, colors[yellow]);

	mov	eax, 4
	imul	ecx, eax, 6
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+36], eax

; 24   :   m_OrangePen = CreatePen(PS_SOLID, 1, colors[orange]);

	mov	edx, 4
	imul	eax, edx, 7
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+32], eax

; 25   :   m_PurplePen = CreatePen(PS_SOLID, 1, colors[purple]);

	mov	eax, 4
	shl	eax, 3
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+40], eax

; 26   :   m_BrownPen = CreatePen(PS_SOLID, 1, colors[brown]);

	mov	eax, 4
	imul	ecx, eax, 9
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+44], eax

; 27   :   
; 28   :   m_DarkGreenPen = CreatePen(PS_SOLID, 1, colors[dark_green]);

	mov	edx, 4
	imul	eax, edx, 11
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+48], eax

; 29   : 
; 30   :   m_LightBluePen = CreatePen(PS_SOLID, 1, colors[light_blue]);

	mov	eax, 4
	imul	ecx, eax, 12
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+52], eax

; 31   :   m_LightGreyPen = CreatePen(PS_SOLID, 1, colors[light_grey]);

	mov	edx, 4
	imul	eax, edx, 13
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+56], eax

; 32   :   m_LightPinkPen = CreatePen(PS_SOLID, 1, colors[light_pink]);

	mov	eax, 4
	imul	ecx, eax, 14
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	1
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+60], eax

; 33   : 
; 34   :   m_ThickBlackPen = CreatePen(PS_SOLID, 2, colors[black]);

	mov	edx, 4
	imul	eax, edx, 3
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	2
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+64], eax

; 35   :   m_ThickWhitePen = CreatePen(PS_SOLID, 2, colors[white]);

	mov	eax, 4
	imul	ecx, eax, 10
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	push	2
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+68], eax

; 36   :   m_ThickRedPen = CreatePen(PS_SOLID, 2, colors[red]);

	mov	edx, 4
	imul	eax, edx, 0
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	2
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+72], eax

; 37   :   m_ThickGreenPen = CreatePen(PS_SOLID, 2, colors[green]);

	mov	eax, 4
	shl	eax, 1
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	2
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+76], eax

; 38   :   m_ThickBluePen = CreatePen(PS_SOLID, 2, colors[blue]);

	mov	eax, 4
	shl	eax, 0
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	push	2
	push	0
	call	DWORD PTR __imp__CreatePen@12
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+80], eax

; 39   : 
; 40   :   m_GreenBrush = CreateSolidBrush(colors[green]);

	mov	eax, 4
	shl	eax, 1
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+92], eax

; 41   :   m_RedBrush   = CreateSolidBrush(colors[red]);

	mov	eax, 4
	imul	ecx, eax, 0
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+88], eax

; 42   :   m_BlueBrush  = CreateSolidBrush(colors[blue]);

	mov	edx, 4
	shl	edx, 0
	mov	esi, esp
	mov	eax, DWORD PTR ?colors@@3QBKB[edx]
	push	eax
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+96], eax

; 43   :   m_GreyBrush  = CreateSolidBrush(colors[grey]);

	mov	edx, 4
	imul	eax, edx, 5
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+100], eax

; 44   :   m_BrownBrush = CreateSolidBrush(colors[brown]);

	mov	eax, 4
	imul	ecx, eax, 9
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+104], eax

; 45   :   m_YellowBrush = CreateSolidBrush(colors[yellow]);

	mov	edx, 4
	imul	eax, edx, 6
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+108], eax

; 46   :   m_LightBlueBrush = CreateSolidBrush(RGB(0,255,255));

	mov	esi, esp
	push	16776960				; 00ffff00H
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+116], eax

; 47   :   m_DarkGreenBrush = CreateSolidBrush(colors[dark_green]);

	mov	edx, 4
	imul	eax, edx, 11
	mov	esi, esp
	mov	ecx, DWORD PTR ?colors@@3QBKB[eax]
	push	ecx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+120], eax

; 48   :   m_OrangeBrush = CreateSolidBrush(colors[orange]);

	mov	eax, 4
	imul	ecx, eax, 7
	mov	esi, esp
	mov	edx, DWORD PTR ?colors@@3QBKB[ecx]
	push	edx
	call	DWORD PTR __imp__CreateSolidBrush@4
	cmp	esi, esp
	call	__RTC_CheckEsp
	mov	ecx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [ecx+112], eax

; 49   : 
; 50   :   m_hdc = NULL;

	mov	edx, DWORD PTR _this$[ebp]
	mov	DWORD PTR [edx+124], 0

; 51   : }

	mov	eax, DWORD PTR _this$[ebp]
	pop	esi
	add	esp, 4
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
??0Cgdi@@AAE@XZ ENDP					; Cgdi::Cgdi
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\utils.h
;	COMDAT ??__EMinFloat@@YAXXZ
text$di	SEGMENT
??__EMinFloat@@YAXXZ PROC				; `dynamic initializer for 'MinFloat'', COMDAT

; 27   : const float   MinFloat  = (std::numeric_limits<float>::min)();

	push	ebp
	mov	ebp, esp
	call	?min@?$numeric_limits@M@std@@SAMXZ	; std::numeric_limits<float>::min
	fstp	DWORD PTR ?MinFloat@@3MB
	cmp	ebp, esp
	call	__RTC_CheckEsp
	pop	ebp
	ret	0
??__EMinFloat@@YAXXZ ENDP				; `dynamic initializer for 'MinFloat''
text$di	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\utils.h
;	COMDAT ??__EMaxFloat@@YAXXZ
text$di	SEGMENT
??__EMaxFloat@@YAXXZ PROC				; `dynamic initializer for 'MaxFloat'', COMDAT

; 26   : const float   MaxFloat  = (std::numeric_limits<float>::max)();

	push	ebp
	mov	ebp, esp
	call	?max@?$numeric_limits@M@std@@SAMXZ	; std::numeric_limits<float>::max
	fstp	DWORD PTR ?MaxFloat@@3MB
	cmp	ebp, esp
	call	__RTC_CheckEsp
	pop	ebp
	ret	0
??__EMaxFloat@@YAXXZ ENDP				; `dynamic initializer for 'MaxFloat''
text$di	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\utils.h
;	COMDAT ??__EMinDouble@@YAXXZ
text$di	SEGMENT
??__EMinDouble@@YAXXZ PROC				; `dynamic initializer for 'MinDouble'', COMDAT

; 25   : const double  MinDouble = (std::numeric_limits<double>::min)();

	push	ebp
	mov	ebp, esp
	call	?min@?$numeric_limits@N@std@@SANXZ	; std::numeric_limits<double>::min
	fstp	QWORD PTR ?MinDouble@@3NB
	cmp	ebp, esp
	call	__RTC_CheckEsp
	pop	ebp
	ret	0
??__EMinDouble@@YAXXZ ENDP				; `dynamic initializer for 'MinDouble''
text$di	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\utils.h
;	COMDAT ??__EMaxDouble@@YAXXZ
text$di	SEGMENT
??__EMaxDouble@@YAXXZ PROC				; `dynamic initializer for 'MaxDouble'', COMDAT

; 24   : const double  MaxDouble = (std::numeric_limits<double>::max)();

	push	ebp
	mov	ebp, esp
	call	?max@?$numeric_limits@N@std@@SANXZ	; std::numeric_limits<double>::max
	fstp	QWORD PTR ?MaxDouble@@3NB
	cmp	ebp, esp
	call	__RTC_CheckEsp
	pop	ebp
	ret	0
??__EMaxDouble@@YAXXZ ENDP				; `dynamic initializer for 'MaxDouble''
text$di	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\utils.h
;	COMDAT ??__EMaxInt@@YAXXZ
text$di	SEGMENT
??__EMaxInt@@YAXXZ PROC					; `dynamic initializer for 'MaxInt'', COMDAT

; 23   : const int     MaxInt    = (std::numeric_limits<int>::max)();

	push	ebp
	mov	ebp, esp
	call	?max@?$numeric_limits@H@std@@SAHXZ	; std::numeric_limits<int>::max
	mov	DWORD PTR ?MaxInt@@3HB, eax
	cmp	ebp, esp
	call	__RTC_CheckEsp
	pop	ebp
	ret	0
??__EMaxInt@@YAXXZ ENDP					; `dynamic initializer for 'MaxInt''
text$di	ENDS
; Function compile flags: /Odtp /RTCsu
; File c:\program files (x86)\microsoft visual studio 14.0\vc\include\limits
;	COMDAT ?max@?$numeric_limits@N@std@@SANXZ
_TEXT	SEGMENT
?max@?$numeric_limits@N@std@@SANXZ PROC			; std::numeric_limits<double>::max, COMDAT

; 1150 : 		{	// return maximum value

	push	ebp
	mov	ebp, esp

; 1151 : 		return (_DBL_MAX);

	fld	QWORD PTR __real@7fefffffffffffff

; 1152 : 		}

	pop	ebp
	ret	0
?max@?$numeric_limits@N@std@@SANXZ ENDP			; std::numeric_limits<double>::max
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File c:\program files (x86)\microsoft visual studio 14.0\vc\include\limits
;	COMDAT ?min@?$numeric_limits@N@std@@SANXZ
_TEXT	SEGMENT
?min@?$numeric_limits@N@std@@SANXZ PROC			; std::numeric_limits<double>::min, COMDAT

; 1145 : 		{	// return minimum value

	push	ebp
	mov	ebp, esp

; 1146 : 		return (_DBL_MIN);

	fld	QWORD PTR __real@0010000000000000

; 1147 : 		}

	pop	ebp
	ret	0
?min@?$numeric_limits@N@std@@SANXZ ENDP			; std::numeric_limits<double>::min
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File c:\program files (x86)\microsoft visual studio 14.0\vc\include\limits
;	COMDAT ?max@?$numeric_limits@M@std@@SAMXZ
_TEXT	SEGMENT
?max@?$numeric_limits@M@std@@SAMXZ PROC			; std::numeric_limits<float>::max, COMDAT

; 1087 : 		{	// return maximum value

	push	ebp
	mov	ebp, esp

; 1088 : 		return (_FLT_MAX);

	fld	DWORD PTR __real@7f7fffff

; 1089 : 		}

	pop	ebp
	ret	0
?max@?$numeric_limits@M@std@@SAMXZ ENDP			; std::numeric_limits<float>::max
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File c:\program files (x86)\microsoft visual studio 14.0\vc\include\limits
;	COMDAT ?min@?$numeric_limits@M@std@@SAMXZ
_TEXT	SEGMENT
?min@?$numeric_limits@M@std@@SAMXZ PROC			; std::numeric_limits<float>::min, COMDAT

; 1082 : 		{	// return minimum value

	push	ebp
	mov	ebp, esp

; 1083 : 		return (_FLT_MIN);

	fld	DWORD PTR __real@00800000

; 1084 : 		}

	pop	ebp
	ret	0
?min@?$numeric_limits@M@std@@SAMXZ ENDP			; std::numeric_limits<float>::min
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File c:\program files (x86)\microsoft visual studio 14.0\vc\include\limits
;	COMDAT ?max@?$numeric_limits@H@std@@SAHXZ
_TEXT	SEGMENT
?max@?$numeric_limits@H@std@@SAHXZ PROC			; std::numeric_limits<int>::max, COMDAT

; 681  : 		{	// return maximum value

	push	ebp
	mov	ebp, esp

; 682  : 		return (INT_MAX);

	mov	eax, 2147483647				; 7fffffffH

; 683  : 		}

	pop	ebp
	ret	0
?max@?$numeric_limits@H@std@@SAHXZ ENDP			; std::numeric_limits<int>::max
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\cgdi.cpp
;	COMDAT ?__empty_global_delete@@YAXPAXI@Z
_TEXT	SEGMENT
___formal$ = 8						; size = 4
___formal$ = 12						; size = 4
?__empty_global_delete@@YAXPAXI@Z PROC			; __empty_global_delete, COMDAT

	push	ebp
	mov	ebp, esp
	pop	ebp
	ret	0
?__empty_global_delete@@YAXPAXI@Z ENDP			; __empty_global_delete
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu
; File e:\documents\cours\git\raven\common\misc\cgdi.cpp
;	COMDAT ?__empty_global_delete@@YAXPAX@Z
_TEXT	SEGMENT
___formal$ = 8						; size = 4
?__empty_global_delete@@YAXPAX@Z PROC			; __empty_global_delete, COMDAT

	push	ebp
	mov	ebp, esp
	pop	ebp
	ret	0
?__empty_global_delete@@YAXPAX@Z ENDP			; __empty_global_delete
_TEXT	ENDS
END
